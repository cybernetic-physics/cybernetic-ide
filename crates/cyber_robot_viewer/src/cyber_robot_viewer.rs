use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use anyhow::{Context as _, Result, anyhow};
use gpui::{
    App, Context, CursorStyle, Entity, EventEmitter, FocusHandle, Focusable, ImageSource,
    IntoElement, MouseButton, MouseDownEvent, MouseMoveEvent, MouseUpEvent, ObjectFit,
    ParentElement, Pixels, Point, Render, RenderImage, ScrollDelta, ScrollWheelEvent, SharedString,
    Styled, Task, Window, actions, img, px, relative, rgb,
};
#[cfg(test)]
use serde::Deserialize;
use serde_json::{Value, json};
use ui::prelude::*;
use util::ResultExt as _;
use workspace::Workspace;
use workspace::item::Item;

const HARNESS_MARKER_PATH: &str = "overlays/unitree-g1-mujoco-protocol/Dockerfile";
const AUTO_OPEN_ENV: &str = "CYBER_ROBOT_VIEWER_OPEN_ON_STARTUP";
const DEFAULT_IMAGE: &str = "cyber/unitree-g1-mujoco-protocol:0.1.0";
const DEFAULT_MODEL_PATH: &str = "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml";
const PHYSICS_URL: &str = "ws://127.0.0.1:8788";
const GAME_CONTROL_URL: &str = "http://127.0.0.1:38383";
const GAME_CONTROL_HOST: &str = "127.0.0.1";
const GAME_CONTROL_PORT: u16 = 38383;
const STATUS_PATH: &str = "/status";
const VISUAL_FRAME_PATH: &str = "/visual_frame";
const CAMERA_CONTROL_PATH: &str = "/camera";
const CAMERA_FRAME_PATH: &str = "/camera_frame_0.jpg";
const STATUS_POLL_INTERVAL: Duration = Duration::from_secs(2);
const FRAME_POLL_INTERVAL: Duration = Duration::from_millis(125);
const MAX_READY_ATTEMPTS: usize = 45;
const CAMERA_DRAG_THRESHOLD_PX: f32 = 4.0;

actions!(
    cyber,
    [
        /// Opens the embedded robot viewer.
        OpenRobotViewer
    ]
);

pub fn init(cx: &mut App) {
    let auto_open = std::env::var_os(AUTO_OPEN_ENV).is_some();
    cx.observe_new(move |workspace: &mut Workspace, window, cx| {
        register_robot_viewer_action(workspace);
        if let (true, Some(window)) = (auto_open, window) {
            open_robot_viewer(workspace, CyberRobotViewer::new, window, cx);
        }
    })
    .detach();
}

fn register_robot_viewer_action(workspace: &mut Workspace) {
    workspace.register_action(|workspace, _: &OpenRobotViewer, window, cx| {
        open_robot_viewer(workspace, CyberRobotViewer::new, window, cx);
    });
}

fn open_robot_viewer(
    workspace: &mut Workspace,
    create_viewer: impl FnOnce(&mut Window, &mut Context<Workspace>) -> Entity<CyberRobotViewer>,
    window: &mut Window,
    cx: &mut Context<Workspace>,
) {
    let existing_view_idx = {
        let active_pane = workspace.active_pane().read(cx);
        active_pane
            .items_of_type::<CyberRobotViewer>()
            .next()
            .and_then(|view| active_pane.index_for_item(&view))
    };

    if let Some(existing_view_idx) = existing_view_idx {
        workspace.active_pane().update(cx, |pane, cx| {
            pane.activate_item(existing_view_idx, true, true, window, cx);
        });
        return;
    }

    let viewer = create_viewer(window, cx);
    workspace.add_item_to_active_pane(Box::new(viewer), None, true, window, cx);
    cx.notify();
}

pub struct CyberRobotViewer {
    focus_handle: FocusHandle,
    config: RobotDockerConfig,
    phase: ViewerPhase,
    status: SharedString,
    telemetry: Option<RobotTelemetry>,
    latest_frame: Option<Arc<RenderImage>>,
    camera_drag: Option<CameraDrag>,
    camera_request: CameraRequestState,
    log_lines: Vec<SharedString>,
    connection_task: Task<()>,
    camera_task: Task<()>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ViewerPhase {
    Idle,
    Launching,
    Connected,
    Waiting,
    Error,
}

impl ViewerPhase {
    fn label(self) -> &'static str {
        match self {
            ViewerPhase::Idle => "Idle",
            ViewerPhase::Launching => "Launching",
            ViewerPhase::Connected => "Connected",
            ViewerPhase::Waiting => "Waiting",
            ViewerPhase::Error => "Error",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CameraDrag {
    last_position: Point<Pixels>,
    mode: CameraDragMode,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct CameraRequestState {
    in_flight: bool,
    pending: Option<CameraCommand>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CameraDragMode {
    Orbit,
    Pan,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum CameraCommand {
    Orbit { dx: f32, dy: f32 },
    Pan { dx: f32, dy: f32 },
    Zoom { delta: f32 },
    Reset,
}

impl CameraCommand {
    fn body(self) -> String {
        match self {
            CameraCommand::Orbit { dx, dy } => {
                json!({ "action": "orbit", "dx": dx, "dy": dy }).to_string()
            }
            CameraCommand::Pan { dx, dy } => {
                json!({ "action": "pan", "dx": dx, "dy": dy }).to_string()
            }
            CameraCommand::Zoom { delta } => {
                json!({ "action": "zoom", "delta": delta }).to_string()
            }
            CameraCommand::Reset => json!({ "action": "reset" }).to_string(),
        }
    }

    fn coalesce(self, next: CameraCommand) -> CameraCommand {
        match (self, next) {
            (
                CameraCommand::Orbit { dx, dy },
                CameraCommand::Orbit {
                    dx: next_dx,
                    dy: next_dy,
                },
            ) => CameraCommand::Orbit {
                dx: dx + next_dx,
                dy: dy + next_dy,
            },
            (
                CameraCommand::Pan { dx, dy },
                CameraCommand::Pan {
                    dx: next_dx,
                    dy: next_dy,
                },
            ) => CameraCommand::Pan {
                dx: dx + next_dx,
                dy: dy + next_dy,
            },
            (CameraCommand::Zoom { delta }, CameraCommand::Zoom { delta: next_delta }) => {
                CameraCommand::Zoom {
                    delta: delta + next_delta,
                }
            }
            (_, CameraCommand::Reset) | (CameraCommand::Reset, _) => next,
            (_, next) => next,
        }
    }
}

impl CyberRobotViewer {
    fn new(window: &mut Window, cx: &mut Context<Workspace>) -> Entity<Self> {
        cx.new(|cx| {
            let mut viewer = Self::new_in_context(cx);
            viewer.start(window, cx);
            viewer
        })
    }

    fn new_in_context(cx: &mut Context<Self>) -> Self {
        Self {
            focus_handle: cx.focus_handle(),
            config: RobotDockerConfig::from_env(),
            phase: ViewerPhase::Idle,
            status: "Ready to launch the robot simulator".into(),
            telemetry: None,
            latest_frame: None,
            camera_drag: None,
            camera_request: CameraRequestState::default(),
            log_lines: Vec::new(),
            connection_task: Task::ready(()),
            camera_task: Task::ready(()),
        }
    }

    #[cfg(test)]
    fn test_connected(cx: &mut Context<Self>) -> Self {
        let mut viewer = Self::new_in_context(cx);
        viewer.phase = ViewerPhase::Connected;
        viewer.status = "Simulator connected".into();
        viewer.telemetry = Some(RobotTelemetry {
            message_type: 6,
            payload_length: 297,
            model_path: Some(DEFAULT_MODEL_PATH.to_string()),
            paused: Some(false),
            actual_speed_factor: Some(1.0),
            robots: vec![RobotSummary {
                name: "g1".to_string(),
                active: true,
                mode: Some("stand".to_string()),
            }],
            visual_frame: None,
            last_probe_at: Some(SystemTime::now()),
        });
        viewer
    }

    #[cfg(test)]
    fn phase(&self) -> ViewerPhase {
        self.phase
    }

    #[cfg(test)]
    fn active_robot_count(&self) -> usize {
        self.telemetry
            .as_ref()
            .map(|telemetry| telemetry.robots.iter().filter(|robot| robot.active).count())
            .unwrap_or_default()
    }

    fn start(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.phase = ViewerPhase::Launching;
        self.status = "Preparing Docker harness".into();
        self.telemetry = None;
        self.latest_frame = None;
        self.camera_drag = None;
        self.camera_request = CameraRequestState::default();
        self.log_lines.clear();
        self.push_log(format!("Harness: {}", self.config.harness_dir.display()));
        self.push_log(format!("Image: {}", self.config.image));
        self.push_log(format!("Physics: {PHYSICS_URL}"));
        self.push_log(format!("GameControl: {GAME_CONTROL_URL}"));
        cx.notify();

        let config = self.config.clone();
        self.connection_task = cx.spawn_in(window, async move |this, cx| {
            update_view(
                &this,
                cx,
                ViewerPhase::Launching,
                "Checking Docker image",
                None,
                None,
            )
            .await;

            if let Err(error) = ensure_image(&config).await {
                update_error(&this, cx, error).await;
                return;
            }

            update_view(
                &this,
                cx,
                ViewerPhase::Launching,
                "Preparing simulator runtime",
                None,
                None,
            )
            .await;
            if let Err(error) = prepare_harness(&config).await {
                update_error(&this, cx, error).await;
                return;
            }

            update_view(
                &this,
                cx,
                ViewerPhase::Launching,
                "Starting Docker Compose service",
                None,
                None,
            )
            .await;
            if let Err(error) = compose_up(&config).await {
                update_error(&this, cx, error).await;
                return;
            }

            for attempt in 1..=MAX_READY_ATTEMPTS {
                let message =
                    format!("Waiting for simulator ports ({attempt}/{MAX_READY_ATTEMPTS})");
                update_view(&this, cx, ViewerPhase::Waiting, message, None, None).await;

                match probe_simulator(&config).await {
                    Ok(telemetry) => {
                        let frame = fetch_camera_frame().await.ok();
                        update_view(
                            &this,
                            cx,
                            ViewerPhase::Connected,
                            "Simulator connected",
                            Some(telemetry),
                            frame,
                        )
                        .await;
                        break;
                    }
                    Err(error) if attempt == MAX_READY_ATTEMPTS => {
                        update_error(&this, cx, error).await;
                        return;
                    }
                    Err(error) => {
                        append_log(&this, cx, format!("Probe pending: {error:#}")).await;
                        cx.background_executor().timer(STATUS_POLL_INTERVAL).await;
                    }
                }
            }

            let mut last_status_probe = Instant::now();
            loop {
                cx.background_executor().timer(FRAME_POLL_INTERVAL).await;
                if let Ok(frame) = fetch_camera_frame().await {
                    update_frame(&this, cx, frame).await;
                }

                if last_status_probe.elapsed() >= STATUS_POLL_INTERVAL {
                    last_status_probe = Instant::now();
                    match probe_simulator(&config).await {
                        Ok(telemetry) => {
                            update_view(
                                &this,
                                cx,
                                ViewerPhase::Connected,
                                "Simulator connected",
                                Some(telemetry),
                                None,
                            )
                            .await;
                        }
                        Err(error) => {
                            update_view(
                                &this,
                                cx,
                                ViewerPhase::Waiting,
                                format!("Simulator probe failed: {error:#}"),
                                None,
                                None,
                            )
                            .await;
                        }
                    }
                }
            }
        });
    }

    fn push_log(&mut self, line: impl Into<SharedString>) {
        self.log_lines.push(line.into());
        if self.log_lines.len() > 12 {
            self.log_lines.remove(0);
        }
    }

    fn start_camera_drag(
        &mut self,
        mode: CameraDragMode,
        event: &MouseDownEvent,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        self.camera_drag = Some(CameraDrag {
            last_position: event.position,
            mode,
        });
        window.prevent_default();
        cx.stop_propagation();
        cx.notify();
    }

    fn stop_camera_drag(
        &mut self,
        _event: &MouseUpEvent,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        self.camera_drag = None;
        window.prevent_default();
        cx.stop_propagation();
        cx.notify();
    }

    fn handle_camera_drag(
        &mut self,
        event: &MouseMoveEvent,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let Some(mut drag) = self.camera_drag else {
            return;
        };
        if event.pressed_button.is_none() {
            self.camera_drag = None;
            cx.notify();
            return;
        }

        let delta = event.position - drag.last_position;
        let dx = delta.x.as_f32();
        let dy = delta.y.as_f32();
        if dx.abs() < CAMERA_DRAG_THRESHOLD_PX && dy.abs() < CAMERA_DRAG_THRESHOLD_PX {
            return;
        }

        drag.last_position = event.position;
        self.camera_drag = Some(drag);
        let command = match drag.mode {
            CameraDragMode::Orbit => CameraCommand::Orbit { dx, dy },
            CameraDragMode::Pan => CameraCommand::Pan { dx, dy },
        };
        self.request_camera_frame(command, window, cx);
        window.prevent_default();
        cx.stop_propagation();
    }

    fn handle_camera_scroll(
        &mut self,
        event: &ScrollWheelEvent,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let delta = scroll_delta_pixels(event.delta);
        let amount = -delta.y.as_f32();
        if amount.abs() < CAMERA_DRAG_THRESHOLD_PX {
            return;
        }

        self.request_camera_frame(CameraCommand::Zoom { delta: amount }, window, cx);
        window.prevent_default();
        cx.stop_propagation();
    }

    fn request_camera_frame(
        &mut self,
        command: CameraCommand,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        if self.camera_request.in_flight {
            self.camera_request.pending = Some(
                self.camera_request
                    .pending
                    .map(|pending| pending.coalesce(command))
                    .unwrap_or(command),
            );
            return;
        }

        self.camera_request.in_flight = true;
        self.camera_task = cx.spawn_in(window, async move |this, cx| {
            match send_camera_command(command).await {
                Ok(()) => {
                    this.update_in(cx, |viewer, window, cx| {
                        viewer.camera_request.in_flight = false;
                        if let Some(pending) = viewer.camera_request.pending.take() {
                            viewer.request_camera_frame(pending, window, cx);
                        }
                        cx.notify();
                    })
                    .log_err();
                }
                Err(error) => {
                    this.update_in(cx, |viewer, window, cx| {
                        viewer.camera_request.in_flight = false;
                        let pending = viewer.camera_request.pending.take();
                        viewer.push_log(format!("Camera control failed: {error:#}"));
                        if let Some(pending) = pending {
                            viewer.request_camera_frame(pending, window, cx);
                        }
                        cx.notify();
                    })
                    .log_err();
                }
            }
        });
    }

    fn render_robot_stage(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let telemetry = self.telemetry.clone();
        let is_connected = self.phase == ViewerPhase::Connected;
        let status_color = if is_connected {
            cx.theme().status().success
        } else if self.phase == ViewerPhase::Error {
            cx.theme().status().error
        } else {
            cx.theme().colors().text_muted
        };

        let model_label = telemetry
            .as_ref()
            .and_then(|telemetry| telemetry.model_path.clone())
            .unwrap_or_else(|| self.config.model_path.clone());
        let mode_label = telemetry
            .as_ref()
            .and_then(|telemetry| telemetry.primary_mode())
            .unwrap_or_else(|| "waiting".to_string());
        let robot_label = telemetry
            .as_ref()
            .and_then(|telemetry| {
                telemetry
                    .visual_frame
                    .as_ref()
                    .and_then(|frame| frame.robot_labels.first())
                    .map(|label| label.name.clone())
                    .or_else(|| {
                        telemetry
                            .robots
                            .iter()
                            .find(|robot| robot.active)
                            .map(|robot| robot.name.clone())
                    })
            })
            .unwrap_or_else(|| "g1".to_string());

        div()
            .id("cyber-robot-viewer-stage")
            .relative()
            .size_full()
            .overflow_hidden()
            .bg(rgb(0x1f3347))
            .cursor(if self.camera_drag.is_some() {
                CursorStyle::ClosedHand
            } else {
                CursorStyle::OpenHand
            })
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(|this, event: &MouseDownEvent, window, cx| {
                    let mode = if event.modifiers.shift {
                        CameraDragMode::Pan
                    } else {
                        CameraDragMode::Orbit
                    };
                    this.start_camera_drag(mode, event, window, cx);
                }),
            )
            .on_mouse_down(
                MouseButton::Right,
                cx.listener(|this, event: &MouseDownEvent, window, cx| {
                    this.start_camera_drag(CameraDragMode::Pan, event, window, cx);
                }),
            )
            .on_mouse_up(
                MouseButton::Left,
                cx.listener(|this, event: &MouseUpEvent, window, cx| {
                    this.stop_camera_drag(event, window, cx);
                }),
            )
            .on_mouse_up(
                MouseButton::Right,
                cx.listener(|this, event: &MouseUpEvent, window, cx| {
                    this.stop_camera_drag(event, window, cx);
                }),
            )
            .on_mouse_up_out(
                MouseButton::Left,
                cx.listener(|this, event: &MouseUpEvent, window, cx| {
                    this.stop_camera_drag(event, window, cx);
                }),
            )
            .on_mouse_up_out(
                MouseButton::Right,
                cx.listener(|this, event: &MouseUpEvent, window, cx| {
                    this.stop_camera_drag(event, window, cx);
                }),
            )
            .on_mouse_move(cx.listener(|this, event: &MouseMoveEvent, window, cx| {
                this.handle_camera_drag(event, window, cx);
            }))
            .on_scroll_wheel(cx.listener(|this, event: &ScrollWheelEvent, window, cx| {
                this.handle_camera_scroll(event, window, cx);
            }))
            .child(
                div()
                    .absolute()
                    .inset_0()
                    .child(render_mujoco_frame(self.latest_frame.clone())),
            )
            .child(self.render_scene_identity(status_color, cx))
            .child(self.render_scene_controls(cx))
            .child(self.render_robot_label(robot_label))
            .child(self.render_metrics_overlay(model_label, mode_label, telemetry, cx))
    }

    fn render_scene_identity(
        &self,
        status_color: gpui::Hsla,
        cx: &mut Context<Self>,
    ) -> impl IntoElement {
        div()
            .absolute()
            .top_4()
            .left_4()
            .flex()
            .items_center()
            .gap_2()
            .px_3()
            .py_2()
            .rounded_md()
            .bg(gpui::black().opacity(0.46))
            .border_1()
            .border_color(gpui::white().opacity(0.16))
            .child(Icon::new(IconName::Server).color(Color::Custom(gpui::white())))
            .child(
                v_flex()
                    .gap_0p5()
                    .child(
                        Label::new("Robot Viewer")
                            .size(LabelSize::Default)
                            .color(Color::Custom(gpui::white())),
                    )
                    .child(
                        h_flex()
                            .gap_1()
                            .items_center()
                            .child(div().size_1p5().rounded_full().bg(status_color))
                            .child(
                                Label::new(self.phase.label())
                                    .size(LabelSize::XSmall)
                                    .color(Color::Custom(gpui::white().opacity(0.78))),
                            )
                            .child(
                                Label::new(self.status.clone())
                                    .size(LabelSize::XSmall)
                                    .color(Color::Custom(gpui::white().opacity(0.72))),
                            ),
                    ),
            )
            .child(
                Icon::new(IconName::ChevronRight).color(Color::Custom(gpui::white().opacity(0.72))),
            )
            .when(matches!(self.phase, ViewerPhase::Error), |this| {
                this.border_color(cx.theme().status().error)
            })
    }

    fn render_scene_controls(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let is_busy = matches!(self.phase, ViewerPhase::Launching | ViewerPhase::Waiting);

        v_flex()
            .absolute()
            .top_4()
            .right_4()
            .gap_2()
            .child(
                div()
                    .p_1()
                    .rounded_md()
                    .bg(gpui::black().opacity(0.46))
                    .border_1()
                    .border_color(gpui::white().opacity(0.16))
                    .child(
                        IconButton::new("cyber-robot-viewer-reconnect", IconName::RotateCw)
                            .icon_color(Color::Custom(gpui::white()))
                            .disabled(is_busy)
                            .on_click(cx.listener(|this, _, window, cx| {
                                this.start(window, cx);
                            })),
                    ),
            )
            .child(self.render_camera_reset_button(cx))
            .child(scene_icon_button(
                "cyber-robot-viewer-pause",
                if self
                    .telemetry
                    .as_ref()
                    .and_then(|telemetry| telemetry.paused)
                    .unwrap_or(false)
                {
                    IconName::PlayOutlined
                } else {
                    IconName::DebugPause
                },
            ))
            .child(scene_icon_button(
                "cyber-robot-viewer-grid",
                IconName::Crosshair,
            ))
    }

    fn render_camera_reset_button(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let is_busy = matches!(self.phase, ViewerPhase::Launching | ViewerPhase::Waiting);

        div()
            .p_1()
            .rounded_md()
            .bg(gpui::black().opacity(0.46))
            .border_1()
            .border_color(gpui::white().opacity(0.16))
            .child(
                IconButton::new("cyber-robot-viewer-reset-camera", IconName::Box)
                    .icon_color(Color::Custom(gpui::white()))
                    .disabled(is_busy)
                    .on_click(cx.listener(|this, _, window, cx| {
                        this.request_camera_frame(CameraCommand::Reset, window, cx);
                    })),
            )
    }

    fn render_robot_label(&self, robot_label: String) -> impl IntoElement {
        div()
            .absolute()
            .top(relative(0.30))
            .left_0()
            .right_0()
            .flex()
            .justify_center()
            .child(
                div()
                    .px_3()
                    .py_1()
                    .rounded_md()
                    .bg(gpui::black().opacity(0.46))
                    .child(
                        Label::new(robot_label)
                            .size(LabelSize::Small)
                            .color(Color::Custom(gpui::white())),
                    ),
            )
    }

    fn render_metrics_overlay(
        &self,
        model_label: String,
        mode_label: String,
        telemetry: Option<RobotTelemetry>,
        _cx: &mut Context<Self>,
    ) -> impl IntoElement {
        let simulation_time = telemetry
            .as_ref()
            .and_then(|telemetry| telemetry.visual_frame.as_ref())
            .and_then(|frame| frame.time)
            .map(|time| format!("{time:.3}s"))
            .unwrap_or_else(|| "waiting".to_string());
        let speed = telemetry
            .as_ref()
            .and_then(|telemetry| telemetry.actual_speed_factor)
            .map(|speed| format!("{speed:.2}x"))
            .unwrap_or_else(|| "waiting".to_string());
        let visual = telemetry
            .as_ref()
            .and_then(|telemetry| telemetry.visual_frame.as_ref())
            .map(|frame| {
                format!(
                    "Frame {} / {} geoms / {} bodies",
                    frame
                        .frame_id
                        .map(|frame_id| frame_id.to_string())
                        .unwrap_or_else(|| "unknown".to_string()),
                    frame.geom_count,
                    frame.body_count
                )
            })
            .unwrap_or_else(|| "Visual frame waiting".to_string());
        let robot_count = telemetry
            .as_ref()
            .map(|telemetry| telemetry.robots.iter().filter(|robot| robot.active).count())
            .unwrap_or_default();

        div()
            .absolute()
            .bottom_4()
            .left_4()
            .max_w(px(430.))
            .px_3()
            .py_2()
            .rounded_md()
            .bg(gpui::white().opacity(0.52))
            .border_1()
            .border_color(gpui::white().opacity(0.42))
            .child(
                v_flex()
                    .gap_1()
                    .child(metric_line("Simulation time", simulation_time))
                    .child(metric_line("Simulation speed", speed))
                    .child(metric_line("Robot mode", mode_label))
                    .child(metric_line("Active robots", robot_count.to_string()))
                    .child(metric_line("Scene", model_label))
                    .child(metric_line("Visual", visual)),
            )
    }
}

impl Render for CyberRobotViewer {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        v_flex()
            .id("CyberRobotViewer")
            .key_context("CyberRobotViewer")
            .track_focus(&self.focus_handle(cx))
            .size_full()
            .bg(cx.theme().colors().editor_background)
            .child(self.render_robot_stage(cx))
    }
}

impl Focusable for CyberRobotViewer {
    fn focus_handle(&self, _cx: &App) -> FocusHandle {
        self.focus_handle.clone()
    }
}

impl EventEmitter<()> for CyberRobotViewer {}

impl Item for CyberRobotViewer {
    type Event = ();

    fn tab_icon(&self, _window: &Window, _cx: &App) -> Option<Icon> {
        Some(Icon::new(IconName::Server))
    }

    fn tab_content_text(&self, _detail: usize, _cx: &App) -> SharedString {
        "Robot Viewer".into()
    }

    fn telemetry_event_text(&self) -> Option<&'static str> {
        Some("cyber robot viewer: open")
    }

    fn to_item_events(_event: &Self::Event, _f: &mut dyn FnMut(workspace::item::ItemEvent)) {}
}

#[derive(Clone, Debug)]
struct RobotDockerConfig {
    harness_dir: PathBuf,
    image: String,
    model_path: String,
}

impl RobotDockerConfig {
    fn from_env() -> Self {
        Self {
            harness_dir: std::env::var("CYBER_ROBOT_HARNESS_DIR")
                .map(PathBuf::from)
                .unwrap_or_else(|_| default_harness_dir()),
            image: std::env::var("CYBER_ROBOT_IMAGE").unwrap_or_else(|_| DEFAULT_IMAGE.to_string()),
            model_path: std::env::var("CYBER_ROBOT_MODEL_PATH")
                .unwrap_or_else(|_| DEFAULT_MODEL_PATH.to_string()),
        }
    }
}

fn default_harness_dir() -> PathBuf {
    std::env::current_dir()
        .ok()
        .and_then(find_harness_root)
        .or_else(|| find_harness_root(PathBuf::from(env!("CARGO_MANIFEST_DIR"))))
        .unwrap_or_else(|| PathBuf::from("."))
}

fn find_harness_root(start: PathBuf) -> Option<PathBuf> {
    start
        .ancestors()
        .find(|ancestor| ancestor.join(HARNESS_MARKER_PATH).exists())
        .map(Path::to_path_buf)
}

#[derive(Clone, Debug, Default, PartialEq)]
struct RobotTelemetry {
    message_type: u64,
    payload_length: u64,
    model_path: Option<String>,
    paused: Option<bool>,
    actual_speed_factor: Option<f64>,
    robots: Vec<RobotSummary>,
    visual_frame: Option<VisualFrameSummary>,
    last_probe_at: Option<SystemTime>,
}

impl RobotTelemetry {
    fn primary_mode(&self) -> Option<String> {
        self.robots
            .iter()
            .find(|robot| robot.active)
            .and_then(|robot| robot.mode.clone())
            .or_else(|| self.robots.first().and_then(|robot| robot.mode.clone()))
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct RobotSummary {
    name: String,
    active: bool,
    mode: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct VisualFrameSummary {
    revision: Option<String>,
    frame_id: Option<u64>,
    time: Option<f64>,
    geom_count: usize,
    body_count: usize,
    camera_count: usize,
    robot_labels: Vec<RobotVisualLabel>,
    robot_body_points: Vec<RobotBodyPoint>,
}

#[derive(Clone, Debug, PartialEq)]
struct RobotVisualLabel {
    name: String,
    body_id: Option<u64>,
    position: [f64; 3],
    yaw: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct RobotBodyPoint {
    body_id: Option<u64>,
    position: [f64; 3],
}

#[cfg(test)]
#[derive(Debug, Deserialize)]
struct ProbeOutput {
    #[serde(rename = "messageType")]
    message_type: u64,
    #[serde(rename = "payloadLength")]
    payload_length: u64,
    decoded: Option<Value>,
}

async fn ensure_image(config: &RobotDockerConfig) -> Result<()> {
    if docker_image_exists(&config.image).await? {
        return Ok(());
    }

    if config.image != DEFAULT_IMAGE {
        anyhow::bail!(
            "Docker image '{}' is not local. Pull/build it first or set CYBER_ROBOT_IMAGE to a local image.",
            config.image
        );
    }

    run_command(
        &config.harness_dir,
        "docker",
        &[
            "build",
            "-t",
            DEFAULT_IMAGE,
            "overlays/unitree-g1-mujoco-protocol",
        ],
        &[],
    )
    .await?;
    Ok(())
}

async fn docker_image_exists(image: &str) -> Result<bool> {
    let output =
        run_command_allow_failure(Path::new("."), "docker", &["image", "inspect", image], &[])
            .await?;
    Ok(output.status_success)
}

async fn prepare_harness(config: &RobotDockerConfig) -> Result<()> {
    run_command(
        &config.harness_dir,
        "node",
        &["script/prepare-unitree-g1-mujoco-container.mjs"],
        &[
            ("UNITREE_G1_MUJOCO_IMAGE", config.image.as_str()),
            ("UNITREE_G1_MODEL_PATH", config.model_path.as_str()),
        ],
    )
    .await?;
    Ok(())
}

async fn compose_up(config: &RobotDockerConfig) -> Result<()> {
    let output = run_command_allow_failure(
        &config.harness_dir,
        "docker",
        &[
            "compose",
            "--env-file",
            ".runtime/unitree-g1-mujoco/compose.env",
            "-f",
            "overlays/unitree-g1-mujoco-container/compose.yaml",
            "up",
            "-d",
        ],
        &[],
    )
    .await?;
    if output.status_success {
        return Ok(());
    }

    let combined_output = format!("{}\n{}", output.stdout, output.stderr);
    if !combined_output.contains("network") || !combined_output.contains("not found") {
        anyhow::bail!(
            "docker compose up failed with status {:?}\nstdout:\n{}\nstderr:\n{}",
            output.status_code,
            output.stdout,
            output.stderr
        );
    }

    run_command_allow_failure(
        &config.harness_dir,
        "docker",
        &[
            "compose",
            "--env-file",
            ".runtime/unitree-g1-mujoco/compose.env",
            "-f",
            "overlays/unitree-g1-mujoco-container/compose.yaml",
            "down",
            "--remove-orphans",
        ],
        &[],
    )
    .await?;
    run_command_allow_failure(
        &config.harness_dir,
        "docker",
        &["rm", "-f", "unitree-g1-mujoco"],
        &[],
    )
    .await?;

    run_command(
        &config.harness_dir,
        "docker",
        &[
            "compose",
            "--env-file",
            ".runtime/unitree-g1-mujoco/compose.env",
            "-f",
            "overlays/unitree-g1-mujoco-container/compose.yaml",
            "up",
            "-d",
        ],
        &[],
    )
    .await?;
    Ok(())
}

async fn probe_simulator(_config: &RobotDockerConfig) -> Result<RobotTelemetry> {
    let status = http_get_json(STATUS_PATH).await?;
    let decoded = status
        .get("simulation")
        .cloned()
        .ok_or_else(|| anyhow!("GameControl status did not contain simulation state"))?;
    let mut telemetry = parse_simulation_state_value(6, 0, decoded);

    if let Ok(visual_frame) = http_get_json(VISUAL_FRAME_PATH).await
        && let Ok(visual_frame) = parse_visual_frame_value(&visual_frame)
    {
        telemetry.visual_frame = Some(visual_frame);
    }

    Ok(telemetry)
}

async fn fetch_camera_frame() -> Result<Arc<RenderImage>> {
    let bytes = http_get_bytes(CAMERA_FRAME_PATH).await?;
    render_image_from_bytes(&bytes)
}

async fn send_camera_command(command: CameraCommand) -> Result<()> {
    let body = command.body();
    http_post_bytes(CAMERA_CONTROL_PATH, body.as_bytes(), "application/json").await?;
    Ok(())
}

fn render_image_from_bytes(bytes: &[u8]) -> Result<Arc<RenderImage>> {
    let format =
        image::guess_format(bytes).context("failed to detect MuJoCo frame image format")?;
    let mut data = image::load_from_memory_with_format(bytes, format)
        .context("failed to decode MuJoCo camera frame")?
        .into_rgba8();

    for pixel in data.chunks_exact_mut(4) {
        pixel.swap(0, 2);
    }

    Ok(Arc::new(RenderImage::new(vec![image::Frame::new(data)])))
}

async fn http_get_json(path: &str) -> Result<Value> {
    let bytes = http_get_bytes(path).await?;
    serde_json::from_slice(&bytes).with_context(|| format!("failed to parse JSON from {path}"))
}

async fn http_get_bytes(path: &str) -> Result<Vec<u8>> {
    http_request("GET", path, &[], None)
}

async fn http_post_bytes(path: &str, body: &[u8], content_type: &str) -> Result<Vec<u8>> {
    http_request("POST", path, body, Some(content_type))
}

fn http_request(
    method: &str,
    path: &str,
    body: &[u8],
    content_type: Option<&str>,
) -> Result<Vec<u8>> {
    let address = format!("{GAME_CONTROL_HOST}:{GAME_CONTROL_PORT}")
        .parse()
        .context("failed to parse GameControl socket address")?;
    let mut stream = TcpStream::connect_timeout(&address, Duration::from_secs(1))
        .with_context(|| format!("failed to connect to {GAME_CONTROL_URL}"))?;
    stream
        .set_read_timeout(Some(Duration::from_secs(3)))
        .context("failed to set GameControl read timeout")?;
    stream
        .set_write_timeout(Some(Duration::from_secs(3)))
        .context("failed to set GameControl write timeout")?;

    let mut request = format!(
        "{method} {path} HTTP/1.1\r\nHost: {GAME_CONTROL_HOST}:{GAME_CONTROL_PORT}\r\nConnection: close\r\nAccept: */*\r\n"
    );
    if let Some(content_type) = content_type {
        request.push_str(&format!("Content-Type: {content_type}\r\n"));
    }
    request.push_str(&format!("Content-Length: {}\r\n\r\n", body.len()));

    stream
        .write_all(request.as_bytes())
        .context("failed to write GameControl HTTP request headers")?;
    if !body.is_empty() {
        stream
            .write_all(body)
            .context("failed to write GameControl HTTP request body")?;
    }

    let mut response = Vec::new();
    stream
        .read_to_end(&mut response)
        .context("failed to read GameControl HTTP response")?;

    let headers_end = response
        .windows(4)
        .position(|window| window == b"\r\n\r\n")
        .ok_or_else(|| anyhow!("GameControl HTTP response did not contain headers"))?;
    let header_bytes = &response[..headers_end];
    let body = response[headers_end + 4..].to_vec();
    let headers = String::from_utf8_lossy(header_bytes);
    let status_line = headers
        .lines()
        .next()
        .ok_or_else(|| anyhow!("GameControl HTTP response did not contain a status line"))?;
    let status_code = status_line
        .split_whitespace()
        .nth(1)
        .and_then(|code| code.parse::<u16>().ok())
        .ok_or_else(|| anyhow!("failed to parse GameControl HTTP status: {status_line}"))?;
    if !(200..300).contains(&status_code) {
        let body_preview = String::from_utf8_lossy(&body);
        anyhow::bail!("GameControl HTTP {method} {path} returned {status_code}: {body_preview}");
    }

    Ok(body)
}

async fn run_command(
    cwd: &Path,
    program: &str,
    args: &[&str],
    envs: &[(&str, &str)],
) -> Result<CommandOutput> {
    let output = run_command_allow_failure(cwd, program, args, envs).await?;
    if output.status_success {
        Ok(output)
    } else {
        anyhow::bail!(
            "{} {} failed with status {:?}\nstdout:\n{}\nstderr:\n{}",
            program,
            args.join(" "),
            output.status_code,
            output.stdout,
            output.stderr
        );
    }
}

async fn run_command_allow_failure(
    cwd: &Path,
    program: &str,
    args: &[&str],
    envs: &[(&str, &str)],
) -> Result<CommandOutput> {
    let mut command = util::command::new_command(program);
    command.current_dir(cwd);
    command.args(args);
    for (key, value) in envs {
        command.env(key, value);
    }
    let output = command
        .output()
        .await
        .with_context(|| format!("failed to run {} {}", program, args.join(" ")))?;
    Ok(CommandOutput {
        status_success: output.status.success(),
        status_code: output.status.code(),
        stdout: String::from_utf8_lossy(&output.stdout).to_string(),
        stderr: String::from_utf8_lossy(&output.stderr).to_string(),
    })
}

struct CommandOutput {
    status_success: bool,
    status_code: Option<i32>,
    stdout: String,
    stderr: String,
}

#[cfg(test)]
fn parse_probe_stdout(stdout: &str) -> Result<ProbeOutput> {
    let json_start = stdout
        .find('{')
        .ok_or_else(|| anyhow!("probe output did not contain JSON"))?;
    serde_json::from_str(&stdout[json_start..]).context("failed to parse simulator probe JSON")
}

#[cfg(test)]
fn parse_simulation_state_probe(probe: ProbeOutput) -> Result<RobotTelemetry> {
    let decoded = probe.decoded.unwrap_or(Value::Null);
    Ok(parse_simulation_state_value(
        probe.message_type,
        probe.payload_length,
        decoded,
    ))
}

fn parse_simulation_state_value(
    message_type: u64,
    payload_length: u64,
    decoded: Value,
) -> RobotTelemetry {
    let robots = extract_robots(&decoded);
    RobotTelemetry {
        message_type,
        payload_length,
        model_path: decoded
            .get("model_path")
            .and_then(Value::as_str)
            .map(ToString::to_string),
        paused: decoded.get("paused").and_then(Value::as_bool),
        actual_speed_factor: decoded.get("actual_speed_factor").and_then(Value::as_f64),
        robots,
        visual_frame: None,
        last_probe_at: Some(SystemTime::now()),
    }
}

#[cfg(test)]
fn parse_simulation_state_probe_stdout(stdout: &str) -> Result<RobotTelemetry> {
    parse_simulation_state_probe(parse_probe_stdout(stdout)?)
}

#[cfg(test)]
fn parse_visual_frame_probe(probe: &ProbeOutput) -> Result<VisualFrameSummary> {
    let decoded = probe
        .decoded
        .as_ref()
        .ok_or_else(|| anyhow!("visual frame probe did not contain a decoded payload"))?;
    parse_visual_frame_value(decoded)
}

fn parse_visual_frame_value(decoded: &Value) -> Result<VisualFrameSummary> {
    let robot_labels = extract_robot_visual_labels(decoded);
    let bodies = extract_body_points(decoded);
    let robot_body_points = filter_robot_body_points(&bodies, &robot_labels);

    Ok(VisualFrameSummary {
        revision: decoded
            .get("revision")
            .and_then(Value::as_str)
            .map(ToString::to_string),
        frame_id: number_as_u64(decoded.get("frame_id")),
        time: decoded.get("time").and_then(Value::as_f64),
        geom_count: decoded
            .get("geoms")
            .and_then(Value::as_array)
            .map_or(0, Vec::len),
        body_count: decoded
            .get("bodies")
            .and_then(Value::as_array)
            .map_or(0, Vec::len),
        camera_count: decoded
            .get("cameras")
            .and_then(Value::as_array)
            .map_or(0, Vec::len),
        robot_labels,
        robot_body_points,
    })
}

fn extract_robots(decoded: &Value) -> Vec<RobotSummary> {
    let statuses = decoded
        .get("robot_statuses")
        .and_then(Value::as_object)
        .map(|status_map| {
            status_map
                .iter()
                .filter_map(|(name, active)| active.as_bool().map(|active| (name.clone(), active)))
                .collect::<BTreeMap<_, _>>()
        })
        .unwrap_or_default();

    let modes = decoded
        .get("robot_modes")
        .and_then(Value::as_object)
        .map(|mode_map| {
            mode_map
                .iter()
                .filter_map(|(name, mode)| {
                    mode.as_str().map(|mode| (name.clone(), mode.to_string()))
                })
                .collect::<BTreeMap<_, _>>()
        })
        .unwrap_or_default();

    let mut names = decoded
        .get("all_robot_names")
        .and_then(Value::as_array)
        .map(|names| {
            names
                .iter()
                .filter_map(Value::as_str)
                .map(ToString::to_string)
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();

    for name in statuses.keys().chain(modes.keys()) {
        if !names.iter().any(|existing| existing == name) {
            names.push(name.clone());
        }
    }

    names.sort();
    names.dedup();

    names
        .into_iter()
        .map(|name| RobotSummary {
            active: statuses.get(&name).copied().unwrap_or(false),
            mode: modes.get(&name).cloned(),
            name,
        })
        .collect()
}

fn extract_robot_visual_labels(decoded: &Value) -> Vec<RobotVisualLabel> {
    decoded
        .get("robotLabels")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|label| {
            let name = label.get("name").and_then(Value::as_str)?.to_string();
            let position = array3(label.get("position"))?;
            let yaw = label.get("rotation").and_then(yaw_from_matrix);
            Some(RobotVisualLabel {
                name,
                body_id: number_as_u64(label.get("bodyId")),
                position,
                yaw,
            })
        })
        .collect()
}

fn extract_body_points(decoded: &Value) -> Vec<RobotBodyPoint> {
    decoded
        .get("bodies")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|body| {
            let position = array3(body.get("position"))?;
            Some(RobotBodyPoint {
                body_id: number_as_u64(body.get("id")),
                position,
            })
        })
        .collect()
}

fn filter_robot_body_points(
    bodies: &[RobotBodyPoint],
    labels: &[RobotVisualLabel],
) -> Vec<RobotBodyPoint> {
    if labels.is_empty() {
        return bodies.iter().take(64).cloned().collect();
    }

    let mut points = bodies
        .iter()
        .filter(|body| {
            labels.iter().any(|label| {
                let dx = body.position[0] - label.position[0];
                let dy = body.position[1] - label.position[1];
                let dz = body.position[2] - label.position[2];
                let horizontal_distance = (dx * dx + dy * dy).sqrt();
                horizontal_distance <= 1.4 && dz.abs() <= 1.8
            })
        })
        .cloned()
        .collect::<Vec<_>>();

    points.sort_by(|left, right| {
        left.position[2]
            .partial_cmp(&right.position[2])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    points
}

fn array3(value: Option<&Value>) -> Option<[f64; 3]> {
    let array = value?.as_array()?;
    Some([
        array.first()?.as_f64()?,
        array.get(1)?.as_f64()?,
        array.get(2)?.as_f64()?,
    ])
}

fn yaw_from_matrix(value: &Value) -> Option<f64> {
    let matrix = value.as_array()?;
    let forward_x = matrix.first()?.as_f64()?;
    let forward_y = matrix.get(3)?.as_f64()?;
    Some(forward_y.atan2(forward_x))
}

fn number_as_u64(value: Option<&Value>) -> Option<u64> {
    value.and_then(Value::as_u64).or_else(|| {
        value
            .and_then(Value::as_i64)
            .and_then(|value| value.try_into().ok())
    })
}

fn scroll_delta_pixels(delta: ScrollDelta) -> Point<Pixels> {
    delta.pixel_delta(px(20.))
}

async fn update_view(
    this: &gpui::WeakEntity<CyberRobotViewer>,
    cx: &mut gpui::AsyncWindowContext,
    phase: ViewerPhase,
    status: impl Into<SharedString>,
    telemetry: Option<RobotTelemetry>,
    frame: Option<Arc<RenderImage>>,
) {
    let status = status.into();
    this.update_in(cx, |viewer, _window, cx| {
        viewer.phase = phase;
        viewer.status = status.clone();
        if let Some(telemetry) = telemetry {
            viewer.telemetry = Some(telemetry);
        }
        if let Some(frame) = frame {
            viewer.latest_frame = Some(frame);
        }
        viewer.push_log(status.clone());
        cx.notify();
    })
    .log_err();
}

async fn update_frame(
    this: &gpui::WeakEntity<CyberRobotViewer>,
    cx: &mut gpui::AsyncWindowContext,
    frame: Arc<RenderImage>,
) {
    this.update(cx, |viewer, cx| {
        viewer.latest_frame = Some(frame.clone());
        cx.notify();
    })
    .log_err();
}

async fn append_log(
    this: &gpui::WeakEntity<CyberRobotViewer>,
    cx: &mut gpui::AsyncWindowContext,
    line: impl Into<SharedString>,
) {
    let line = line.into();
    this.update(cx, |viewer, cx| {
        viewer.push_log(line.clone());
        cx.notify();
    })
    .log_err();
}

async fn update_error(
    this: &gpui::WeakEntity<CyberRobotViewer>,
    cx: &mut gpui::AsyncWindowContext,
    error: anyhow::Error,
) {
    let message = format!("{error:#}");
    this.update(cx, |viewer, cx| {
        viewer.phase = ViewerPhase::Error;
        viewer.status = message.clone().into();
        viewer.push_log(message);
        cx.notify();
    })
    .log_err();
}

fn metric_line(label: impl Into<SharedString>, value: impl Into<SharedString>) -> impl IntoElement {
    h_flex()
        .gap_4()
        .justify_between()
        .items_center()
        .child(
            Label::new(label.into())
                .size(LabelSize::XSmall)
                .color(Color::Custom(gpui::black().opacity(0.68))),
        )
        .child(
            Label::new(value.into())
                .size(LabelSize::XSmall)
                .color(Color::Custom(gpui::black().opacity(0.9))),
        )
}

fn scene_icon_button(id: &'static str, icon: IconName) -> impl IntoElement {
    div()
        .p_1()
        .rounded_md()
        .bg(gpui::black().opacity(0.46))
        .border_1()
        .border_color(gpui::white().opacity(0.16))
        .child(
            IconButton::new(id, icon)
                .icon_color(Color::Custom(gpui::white()))
                .disabled(true),
        )
}

fn render_mujoco_frame(frame: Option<Arc<RenderImage>>) -> impl IntoElement {
    match frame {
        Some(frame) => div().size_full().child(
            img(ImageSource::Render(frame))
                .size_full()
                .object_fit(ObjectFit::Cover),
        ),
        None => div()
            .size_full()
            .flex()
            .items_center()
            .justify_center()
            .bg(rgb(0x1f3347))
            .child(
                Label::new("Waiting for MuJoCo camera frame")
                    .size(LabelSize::Small)
                    .color(Color::Custom(gpui::white().opacity(0.76))),
            ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use fs::FakeFs;
    use project::Project;

    #[gpui::test]
    async fn embeds_robot_viewer_in_active_workspace_pane(cx: &mut gpui::TestAppContext) {
        cx.update(|cx| {
            workspace::AppState::test(cx);
        });

        let fs = FakeFs::new(cx.executor());
        let project = Project::test(fs, [], cx).await;
        let (workspace, cx) =
            cx.add_window_view(|window, cx| Workspace::test_new(project, window, cx));

        workspace.update_in(cx, |workspace, window, cx| {
            open_robot_viewer(
                workspace,
                |_, cx| cx.new(CyberRobotViewer::test_connected),
                window,
                cx,
            );
        });
        cx.run_until_parked();

        let viewer = workspace.read_with(cx, |workspace, cx| {
            workspace
                .active_pane()
                .read(cx)
                .items_of_type::<CyberRobotViewer>()
                .next()
                .expect("robot viewer should be added to the active pane")
        });

        viewer.read_with(cx, |viewer, _| {
            assert_eq!(viewer.phase(), ViewerPhase::Connected);
            assert_eq!(viewer.active_robot_count(), 1);
        });
        assert_eq!(
            viewer.read_with(cx, |viewer, cx| viewer.tab_content_text(0, cx)),
            SharedString::from("Robot Viewer")
        );
        assert_eq!(
            workspace.read_with(cx, |workspace, cx| {
                workspace
                    .active_pane()
                    .read(cx)
                    .items_of_type::<CyberRobotViewer>()
                    .count()
            }),
            1,
            "the viewer should be a workspace item, not a separate window"
        );

        workspace.update_in(cx, |workspace, window, cx| {
            open_robot_viewer(
                workspace,
                |_, cx| cx.new(CyberRobotViewer::test_connected),
                window,
                cx,
            );
        });

        assert_eq!(
            workspace.read_with(cx, |workspace, cx| {
                workspace
                    .active_pane()
                    .read(cx)
                    .items_of_type::<CyberRobotViewer>()
                    .count()
            }),
            1,
            "reopening should activate the existing embedded viewer"
        );
    }

    #[test]
    fn default_harness_dir_finds_checked_in_runtime() {
        let harness_dir = default_harness_dir();
        assert!(
            harness_dir.join(HARNESS_MARKER_PATH).exists(),
            "expected {} under {}",
            HARNESS_MARKER_PATH,
            harness_dir.display()
        );
    }

    #[test]
    fn parses_simulation_state_probe_output() {
        let stdout = r#"{
  "physicsUrl": "ws://127.0.0.1:8788",
  "topic": "simulation_state",
  "command": null,
  "messageType": 6,
  "payloadLength": 145,
  "decoded": {
    "actual_speed_factor": 1,
    "paused": false,
    "robot_statuses": {"robot1": true, "robot2": false},
    "is_multi_robot": true,
    "robot_modes": {"robot1": "stand", "robot2": "stand"},
    "model_path": "mjcf/football_pitch_T1.xml",
    "all_robot_names": ["robot1", "robot2"]
  },
  "decodeError": null
}"#;

        let telemetry =
            parse_simulation_state_probe_stdout(stdout).expect("probe output should parse");
        assert_eq!(telemetry.message_type, 6);
        assert_eq!(telemetry.payload_length, 145);
        assert_eq!(
            telemetry.model_path.as_deref(),
            Some("mjcf/football_pitch_T1.xml")
        );
        assert_eq!(telemetry.paused, Some(false));
        assert_eq!(telemetry.actual_speed_factor, Some(1.0));
        assert_eq!(telemetry.visual_frame, None);
        assert_eq!(
            telemetry.robots,
            vec![
                RobotSummary {
                    name: "robot1".to_string(),
                    active: true,
                    mode: Some("stand".to_string())
                },
                RobotSummary {
                    name: "robot2".to_string(),
                    active: false,
                    mode: Some("stand".to_string())
                }
            ]
        );
    }

    #[test]
    fn parses_visual_frame_probe_output() {
        let stdout = r#"{
  "physicsUrl": "ws://127.0.0.1:8788",
  "topic": "visual_frame",
  "command": null,
  "messageType": 11,
  "payloadLength": 512,
  "decoded": {
    "revision": "abc",
    "time": 12.5,
    "frame_id": 44,
    "geoms": [{ "id": 1 }],
    "bodies": [
      { "id": 1, "position": [96.0, 5.0, 0.7] },
      { "id": 2, "position": [96.2, 5.0, 1.0] },
      { "id": 25, "position": [-7.0, 0.0, 0.0] }
    ],
    "cameras": [{ "cameraId": 0 }],
    "robotLabels": [
      {
        "bodyId": 1,
        "name": "robot1",
        "position": [96.0, 5.0, 1.3],
        "rotation": [0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      }
    ]
  },
  "decodeError": null
}"#;

        let probe = parse_probe_stdout(stdout).expect("probe output should parse");
        let visual_frame = parse_visual_frame_probe(&probe).expect("visual frame should parse");

        assert_eq!(visual_frame.revision.as_deref(), Some("abc"));
        assert_eq!(visual_frame.frame_id, Some(44));
        assert_eq!(visual_frame.time, Some(12.5));
        assert_eq!(visual_frame.geom_count, 1);
        assert_eq!(visual_frame.body_count, 3);
        assert_eq!(visual_frame.camera_count, 1);
        assert_eq!(visual_frame.robot_labels.len(), 1);
        assert_eq!(visual_frame.robot_labels[0].name, "robot1");
        assert_eq!(visual_frame.robot_labels[0].body_id, Some(1));
        assert_eq!(visual_frame.robot_body_points.len(), 2);
    }

    #[test]
    fn extracts_robots_when_all_robot_names_is_missing() {
        let decoded = serde_json::json!({
            "robot_statuses": {"robot2": true},
            "robot_modes": {"robot1": "stand"}
        });
        let robots = extract_robots(&decoded);
        assert_eq!(
            robots,
            vec![
                RobotSummary {
                    name: "robot1".to_string(),
                    active: false,
                    mode: Some("stand".to_string())
                },
                RobotSummary {
                    name: "robot2".to_string(),
                    active: true,
                    mode: None
                }
            ]
        );
    }
}
