
import os
import json
import math
import time

try:
    import traci
except Exception as e:
    raise RuntimeError("Could not import traci. Make sure SUMO is installed and SUMO_HOME/PYTHONPATH are configured.") from e

try:
    import imageio.v2 as imageio
except Exception:
    imageio = None

try:
    from PIL import Image, ImageDraw, ImageFont, ImageOps
except Exception:
    Image = None
    ImageDraw = None
    ImageFont = None
    ImageOps = None

REPLAY_JSON = "replay_data.json"
SUMO_CONFIG = "simulation.sumocfg"

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

def clean_png_folder(path):
    ensure_dir(path)
    for fname in os.listdir(path):
        if fname.lower().endswith(".png"):
            try:
                os.remove(os.path.join(path, fname))
            except OSError:
                pass

def safe_float(x, default=0.0):
    try:
        if x is None:
            return default
        if isinstance(x, float) and math.isnan(x):
            return default
        return float(x)
    except Exception:
        return default

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def load_replay():
    with open(REPLAY_JSON, "r", encoding="utf-8") as f:
        return json.load(f)

def get_cfg(replay):
    return dict(replay.get("visual_config", {}))

def lane_to_sumo(lane):
    return max(0, int(lane) - 1)

def lane_center_y(lane, lane_width):
    return (int(lane) - 1) * float(lane_width)

def road_y_bounds(replay):
    lane_width = float(replay.get("lane_width", 5.0))
    n_lanes = int(replay.get("n_lanes", 1))
    y_low = -0.5 * lane_width
    y_high = (n_lanes - 1) * lane_width + 0.5 * lane_width
    return y_low, y_high

def vehicle_exists(traj, k):
    exists = traj.get("exists", [])
    return k < len(exists) and int(exists[k]) == 1

def vehicle_value(traj, key, k, default=None):
    arr = traj.get(key, [])
    if k < len(arr):
        return arr[k]
    return default

def set_vehicle_modes(vid):
    try:
        traci.vehicle.setSpeedMode(vid, 0)
        traci.vehicle.setLaneChangeMode(vid, 0)
    except Exception:
        pass

def color_vehicle(vid, traj):
    try:
        if vid == "ego":
            traci.vehicle.setColor(vid, (255, 35, 35, 255))
        elif str(traj.get("kind", "")).lower() == "front":
            traci.vehicle.setColor(vid, (35, 95, 255, 255))
        elif str(traj.get("kind", "")).lower() == "rear":
            traci.vehicle.setColor(vid, (30, 170, 255, 255))
        else:
            traci.vehicle.setColor(vid, (0, 110, 255, 255))
    except Exception:
        pass

def ensure_vehicle(vid, traj, k, replay):
    if not vehicle_exists(traj, k):
        return False
    lane = int(vehicle_value(traj, "lane", k, 1) or 1)
    s = safe_float(vehicle_value(traj, "s", k, 0.0), 0.0)
    v = max(0.0, safe_float(vehicle_value(traj, "v", k, 0.0), 0.0))
    if vid not in traci.vehicle.getIDList():
        vtype = traj.get("type", "frontTrafficType")
        if str(traj.get("kind", "")).lower() == "rear":
            vtype = "rearTrafficType"
        if vid == "ego":
            vtype = "egoType"
        try:
            traci.vehicle.add(vehID=vid, routeID="route0", typeID=vtype, departLane=str(lane_to_sumo(lane)), departPos=str(max(0.0, s)), departSpeed=str(v))
        except Exception:
            return False
        set_vehicle_modes(vid)
        color_vehicle(vid, traj)
    return True

def replay_step(k, replay):
    edge_id = replay.get("edge_id", "E0")
    road_length = float(replay.get("road_length", 1000.0))
    lane_width = float(replay.get("lane_width", 5.0))
    active = set(traci.vehicle.getIDList())
    needed = set()
    for vid, traj in replay.get("vehicles", {}).items():
        if vehicle_exists(traj, k):
            needed.add(vid)
            if ensure_vehicle(vid, traj, k, replay):
                lane = int(vehicle_value(traj, "lane", k, 1) or 1)
                s = clamp(safe_float(vehicle_value(traj, "s", k, 0.0), 0.0), 0.0, road_length - 2.0)
                v = max(0.0, safe_float(vehicle_value(traj, "v", k, 0.0), 0.0))
                y = lane_center_y(lane, lane_width)
                lane_index = lane_to_sumo(lane)
                try:
                    traci.vehicle.moveToXY(vehID=vid, edgeID=edge_id, lane=lane_index, x=s, y=y, angle=90.0, keepRoute=2)
                except Exception:
                    try:
                        traci.vehicle.moveTo(vehID=vid, laneID=f"{edge_id}_{lane_index}", pos=s)
                    except Exception:
                        pass
                try:
                    traci.vehicle.setSpeed(vid, v)
                except Exception:
                    pass
    for vid in active:
        if vid not in needed and (vid == "ego" or vid.startswith("front_") or vid.startswith("rear_")):
            try:
                traci.vehicle.remove(vid)
            except Exception:
                pass

def remove_polygons(poly_ids):
    for pid in list(poly_ids):
        try:
            traci.polygon.remove(pid)
        except Exception:
            pass
    poly_ids.clear()

def add_poly(pid, shape, rgba, layer=100, fill=True, active_poly_ids=None):
    try:
        traci.polygon.add(polygonID=pid, shape=shape, color=rgba, fill=fill, polygonType="dynamic_visual", layer=layer)
        if active_poly_ids is not None:
            active_poly_ids.append(pid)
        return True
    except Exception:
        return False

def rect_shape(x0, x1, y0, y1):
    return [(float(x0), float(y0)), (float(x1), float(y0)), (float(x1), float(y1)), (float(x0), float(y1))]

def add_rect(pid, x0, x1, y0, y1, rgba, layer=100, active_poly_ids=None):
    return add_poly(pid, rect_shape(x0, x1, y0, y1), rgba, layer=layer, fill=True, active_poly_ids=active_poly_ids)

def signal_state(sig, k):
    G = int(sig.get("G", {}).get(str(k), 0))
    candidate_steps = [int(x) for x in sig.get("candidate_steps", [])]
    Gamma = int(sig.get("Gamma", {}).get(str(k), G if k in candidate_steps else 0))
    chosen = sig.get("chosen_step", None)
    qgo = safe_float(sig.get("qGoSIG", {}).get(str(k), 0.0), 0.0)
    return G, Gamma, chosen, qgo

def nearest_upcoming_event(k, replay):
    ego = replay.get("vehicles", {}).get("ego", {})
    ego_s = safe_float(vehicle_value(ego, "s", k, 0.0), 0.0)
    candidates = []
    for ev in replay.get("event_order", []):
        d = safe_float(ev.get("x", 0.0), 0.0) - ego_s
        if d >= -20.0:
            candidates.append((d, ev))
    if not candidates:
        return None, None
    return min(candidates, key=lambda z: z[0])

def update_dynamic_visuals(active_poly_ids, k, replay, cfg):
    remove_polygons(active_poly_ids)
    lane_width = float(replay.get("lane_width", 5.0))
    y_low, y_high = road_y_bounds(replay)
    ego = replay.get("vehicles", {}).get("ego", {})
    ego_s = safe_float(vehicle_value(ego, "s", k, 0.0), 0.0)
    ego_lane = int(vehicle_value(ego, "lane", k, 1) or 1)
    ego_y = lane_center_y(ego_lane, lane_width)
    add_rect(f"dyn_ego_halo_{k}", ego_s - 4.0, ego_s + 8.0, ego_y - 1.65, ego_y + 1.65, (255, 0, 0, 70), layer=160, active_poly_ids=active_poly_ids)
    add_rect(f"dyn_ego_corridor_{k}", ego_s, ego_s + 35.0, ego_y - 0.45, ego_y + 0.45, (255, 40, 40, 45), layer=145, active_poly_ids=active_poly_ids)
    for sid, sig in replay.get("signalized_intersections", {}).items():
        x0 = safe_float(sig.get("s_min", 0.0), 0.0)
        x1 = safe_float(sig.get("s_max", x0 + 10.0), x0 + 10.0)
        x_app = safe_float(sig.get("s_app", max(0, x0 - 80.0)), max(0, x0 - 80.0))
        G, Gamma, chosen, qgo = signal_state(sig, k)
        is_green = G == 1
        glow = (0, 230, 80, 100) if is_green else (255, 30, 30, 115)
        add_rect(f"dyn_SIG_{sid}_stopbar_glow_{k}", x0 - 1.4, x0 + 1.4, y_low - 0.4, y_high + 0.4, glow, layer=170, active_poly_ids=active_poly_ids)
        if Gamma == 1:
            add_rect(f"dyn_SIG_{sid}_gamma_clear_{k}", x0, x1, y_low - 1.0, y_high + 1.0, (0, 220, 90, 55), layer=125, active_poly_ids=active_poly_ids)
        if chosen is not None and int(chosen) == int(k):
            add_rect(f"dyn_SIG_{sid}_chosen_entry_{k}", x0 - 2.1, x0 + 2.1, y_low - 1.2, y_high + 1.2, (255, 215, 0, 160), layer=190, active_poly_ids=active_poly_ids)
        head_x0 = x0 - 7.5
        head_x1 = x0 + 7.5
        head_y0 = y_high + 8.8
        head_y1 = y_high + 14.5
        add_rect(f"dyn_SIG_{sid}_panel_black_{k}", head_x0, head_x1, head_y0, head_y1, (20, 20, 20, 245), layer=220, active_poly_ids=active_poly_ids)
        if is_green:
            add_rect(f"dyn_SIG_{sid}_red_dark_{k}", head_x0 + 1.0, head_x0 + 5.0, head_y0 + 0.8, head_y1 - 0.8, (80, 0, 0, 210), layer=225, active_poly_ids=active_poly_ids)
            add_rect(f"dyn_SIG_{sid}_green_lamp_{k}", head_x1 - 5.0, head_x1 - 1.0, head_y0 + 0.8, head_y1 - 0.8, (0, 255, 60, 255), layer=226, active_poly_ids=active_poly_ids)
            add_rect(f"dyn_SIG_{sid}_state_banner_green_{k}", x0 - 22.0, x0 + 22.0, y_high + 15.2, y_high + 19.2, (0, 170, 50, 235), layer=230, active_poly_ids=active_poly_ids)
        else:
            add_rect(f"dyn_SIG_{sid}_red_lamp_{k}", head_x0 + 1.0, head_x0 + 5.0, head_y0 + 0.8, head_y1 - 0.8, (255, 0, 0, 255), layer=226, active_poly_ids=active_poly_ids)
            add_rect(f"dyn_SIG_{sid}_green_dark_{k}", head_x1 - 5.0, head_x1 - 1.0, head_y0 + 0.8, head_y1 - 0.8, (0, 70, 20, 210), layer=225, active_poly_ids=active_poly_ids)
            add_rect(f"dyn_SIG_{sid}_state_banner_red_{k}", x0 - 22.0, x0 + 22.0, y_high + 15.2, y_high + 19.2, (220, 0, 0, 235), layer=230, active_poly_ids=active_poly_ids)
        if x_app - 15.0 <= ego_s <= x0 + 25.0:
            add_rect(f"dyn_SIG_{sid}_approach_attention_{k}", x_app, x0, y_low - 1.4, y_high + 1.4, (255, 255, 0, 45), layer=128, active_poly_ids=active_poly_ids)
    for nid, ns in replay.get("intersections", {}).items():
        x0 = safe_float(ns.get("s_min", 0.0), 0.0)
        x1 = safe_float(ns.get("s_max", x0 + 10.0), x0 + 10.0)
        wns = ns.get("wNS", [])
        active = k < len(wns) and safe_float(wns[k], 0.0) > 0.5
        if active:
            add_rect(f"dyn_NS_{nid}_active_stop_{k}", x0 - 1.5, x1 + 1.5, y_low - 1.2, y_high + 1.2, (255, 210, 0, 135), layer=172, active_poly_ids=active_poly_ids)
        chosen = ns.get("chosen_step", None)
        if chosen is not None and int(chosen) == int(k):
            add_rect(f"dyn_NS_{nid}_chosen_{k}", x0 - 2.0, x0 + 2.0, y_low - 1.3, y_high + 1.3, (255, 120, 0, 165), layer=190, active_poly_ids=active_poly_ids)
    for ev in replay.get("emergency_events", []):
        kk = int(ev.get("k", -999))
        if abs(kk - k) <= 1:
            x = safe_float(ev.get("s", ego_s), ego_s)
            add_rect(f"dyn_emergency_{kk}_{k}", x - 2.0, x + 2.0, y_low - 2.5, y_high + 2.5, (255, 0, 0, 150), layer=200, active_poly_ids=active_poly_ids)

def get_view_id():
    try:
        ids = traci.gui.getIDList()
        return ids[0] if ids else "View #0"
    except Exception:
        return "View #0"

def update_camera(view_id, k, replay, cfg):
    if view_id is None:
        return
    ego = replay.get("vehicles", {}).get("ego", {})
    ego_s = safe_float(vehicle_value(ego, "s", k, 0.0), 0.0)
    y_low, y_high = road_y_bounds(replay)
    d, ev = nearest_upcoming_event(k, replay)
    if ev is not None and d is not None and d < 180.0:
        event_x = safe_float(ev.get("x", ego_s), ego_s)
        x_center = 0.62 * ego_s + 0.38 * event_x + float(cfg.get("view_ahead_m", 20.0))
    else:
        x_center = ego_s + float(cfg.get("view_ahead_m", 20.0))
    # Center the roadway vertically in the SUMO view.  Some SUMO builds render
    # top-down screenshots with extra vertical whitespace, so we expose a small
    # y-offset knob and still correct the final saved frames in post-processing.
    y_center = 0.5 * (y_low + y_high) + float(cfg.get("camera_y_shift_m", 0.0))
    try:
        traci.gui.setOffset(view_id, x_center, y_center)
    except Exception:
        pass
    try:
        if cfg.get("dynamic_zoom", True):
            base = float(cfg.get("fixed_zoom", 900.0))
            if ev is not None and d is not None:
                target = clamp(base + 2.0 * max(0.0, 120.0 - min(d, 120.0)), float(cfg.get("min_zoom", 650.0)), float(cfg.get("max_zoom", 1500.0)))
            else:
                target = base
            traci.gui.setZoom(view_id, target)
        else:
            traci.gui.setZoom(view_id, float(cfg.get("fixed_zoom", 900.0)))
    except Exception:
        pass

def current_status_text(k, replay):
    dt = float(replay.get("dt", 1.0))
    ego = replay.get("vehicles", {}).get("ego", {})
    ego_s = safe_float(vehicle_value(ego, "s", k, 0.0), 0.0)
    ego_v = safe_float(vehicle_value(ego, "v", k, 0.0), 0.0)
    ego_lane = int(vehicle_value(ego, "lane", k, 1) or 1)
    d, ev = nearest_upcoming_event(k, replay)
    base = f"k={k}   t={k*dt:.1f}s   ego lane={ego_lane}   v={ego_v:.1f} m/s"
    if ev is None:
        return base + "   next event: none"
    if ev.get("kind") == "SIG":
        sig = replay.get("signalized_intersections", {}).get(str(ev.get("id")), {})
        G, Gamma, chosen, qgo = signal_state(sig, k)
        state = "GREEN" if G == 1 else "RED"
        safe = "safe-to-clear" if Gamma == 1 else "not safe-to-clear"
        return base + f"   next SIG {ev.get('id')}: {state}, {safe}, distance={d:.1f} m"
    return base + f"   next NS {ev.get('id')}: stop/yield control, distance={d:.1f} m"

def add_frame_annotation(img, text, replay, k):
    if ImageDraw is None:
        return img
    img = img.convert("RGB")
    draw = ImageDraw.Draw(img)
    try:
        font_big = ImageFont.truetype("arial.ttf", 28)
        font_small = ImageFont.truetype("arial.ttf", 20)
    except Exception:
        font_big = ImageFont.load_default()
        font_small = ImageFont.load_default()
    w, h = img.size
    panel_h = 58
    draw.rectangle((0, 0, w, panel_h), fill=(248, 248, 248), outline=(30, 30, 30))
    draw.text((18, 16), text, fill=(0, 0, 0), font=font_big)
    x = w - 350
    y = 70
    for sid, sig in replay.get("signalized_intersections", {}).items():
        G, Gamma, chosen, qgo = signal_state(sig, k)
        color = (0, 165, 55) if G == 1 else (210, 0, 0)
        label = f"SIG {sid}: {'GREEN' if G == 1 else 'RED'}"
        draw.rectangle((x, y, w - 20, y + 42), fill=color, outline=(20, 20, 20))
        draw.text((x + 12, y + 10), label, fill=(255, 255, 255), font=font_small)
        y += 48
    return img

def _detect_road_band(img, cfg):
    """
    Detect the main dark asphalt road band in a SUMO screenshot.

    The previous crop used background-color differences, which often included a
    huge pale-green landscape rectangle.  This detector looks for the actual
    dark road/asphalt pixels and returns a vertical crop band centered around
    the lanes.  This is the key step that makes saved frames and videos use the
    same effective zoom you see in the GUI.
    """
    img = img.convert("RGB")
    w, h = img.size
    px = img.load()
    dark_threshold = int(cfg.get("road_dark_threshold", 95))
    sample_x_step = max(1, w // 420)
    road_rows = []

    for y in range(h):
        dark_count = 0
        total_count = 0
        for x in range(0, w, sample_x_step):
            r, g, b = px[x, y]
            # Asphalt/road/cross-street pixels are dark and low-saturation.
            if r < dark_threshold and g < dark_threshold and b < dark_threshold:
                dark_count += 1
            total_count += 1
        if total_count > 0 and dark_count / total_count > 0.025:
            road_rows.append(y)

    if not road_rows:
        return None

    y_min = min(road_rows)
    y_max = max(road_rows)
    road_center = int(round(0.5 * (y_min + y_max)))

    # Crop height is intentionally larger than the road band so that signal
    # heads, labels, crosswalks, and nearby vehicles remain visible.
    desired_h = int(float(cfg.get("road_crop_height_ratio", 0.42)) * h)
    desired_h = max(desired_h, int(cfg.get("min_road_crop_px", 420)))
    desired_h = min(desired_h, h)

    expand = int(cfg.get("road_band_expand_px", 90))
    desired_h = max(desired_h, (y_max - y_min + 1) + 2 * expand)
    desired_h = min(desired_h, h)

    y0 = road_center - desired_h // 2
    y1 = y0 + desired_h
    if y0 < 0:
        y0 = 0
        y1 = desired_h
    if y1 > h:
        y1 = h
        y0 = h - desired_h
    return max(0, y0), min(h, y1)


def smart_crop_image(img, bg_rgb=(214, 226, 214), pad=35, cfg=None):
    """
    Center the roadway/lane band in the saved frame.

    If center_lanes_in_frames=True, use a road-band detector rather than a
    background detector.  This prevents the final processed frames/video from
    looking zoomed out or vertically shifted even when the SUMO GUI screenshot
    contains large empty landscape regions.
    """
    if cfg is None:
        cfg = {}
    if ImageOps is None:
        return img
    img = img.convert("RGB")

    if cfg.get("center_lanes_in_frames", True):
        band = _detect_road_band(img, cfg)
        if band is not None:
            y0, y1 = band
            return img.crop((0, y0, img.size[0], y1))

    # Fallback: old background-difference crop.
    px = img.load()
    w, h = img.size
    threshold = 18
    ys = []
    for y in range(h):
        row_has_content = False
        for x in range(0, w, max(1, w // 300)):
            r, g, b = px[x, y]
            if abs(r - bg_rgb[0]) + abs(g - bg_rgb[1]) + abs(b - bg_rgb[2]) > threshold:
                row_has_content = True
                break
        if row_has_content:
            ys.append(y)
    if not ys:
        return img
    y0 = max(0, min(ys) - pad)
    y1 = min(h, max(ys) + pad)
    if y1 <= y0 + 50:
        return img
    return img.crop((0, y0, w, y1))

def resize_with_padding(img, out_w, out_h):
    if ImageOps is None:
        return img.resize((out_w, out_h))
    img = img.convert("RGB")
    img.thumbnail((out_w, out_h), Image.LANCZOS)
    canvas = Image.new("RGB", (out_w, out_h), (235, 235, 235))
    x = (out_w - img.size[0]) // 2
    y = (out_h - img.size[1]) // 2
    canvas.paste(img, (x, y))
    return canvas

def postprocess_frames(raw_dir, processed_dir, replay, cfg):
    if Image is None:
        print("[warning] Pillow is not installed. Skipping post-processing.")
        return raw_dir
    clean_png_folder(processed_dir)
    raw_files = sorted(f for f in os.listdir(raw_dir) if f.lower().endswith(".png"))
    if len(raw_files) == 0:
        print("[warning] No raw frames found.")
        return raw_dir
    for idx, fname in enumerate(raw_files):
        src = os.path.join(raw_dir, fname)
        dst = os.path.join(processed_dir, f"frame_{idx:04d}.png")
        img = Image.open(src).convert("RGB")
        if cfg.get("smart_crop", True):
            img = smart_crop_image(img, bg_rgb=tuple(cfg.get("background_rgb", [214, 226, 214])), pad=int(cfg.get("crop_pad_px", 35)), cfg=cfg)
        if cfg.get("annotate_frames", True):
            img = add_frame_annotation(img, current_status_text(idx, replay), replay, idx)
        if cfg.get("resize_output", True):
            img = resize_with_padding(img, int(cfg.get("output_width", 1700)), int(cfg.get("output_height", 620)))
        img.save(dst)
    return processed_dir

def make_video_from_frames(frames_dir, video_path, fps=2, cfg=None):
    if cfg is None:
        cfg = {}
    if imageio is None:
        print("[warning] imageio is not installed. MP4 video was not created.")
        print("Install with: pip install imageio imageio-ffmpeg")
        return
    frame_files = sorted(f for f in os.listdir(frames_dir) if f.lower().endswith(".png"))
    if len(frame_files) == 0:
        print("[warning] No PNG frames found for video.")
        return

    # Remove old MP4 first so users do not accidentally open a stale video.
    try:
        if os.path.exists(video_path):
            os.remove(video_path)
    except OSError:
        pass

    images = [imageio.imread(os.path.join(frames_dir, fname)) for fname in frame_files]
    try:
        imageio.mimsave(
            video_path,
            images,
            fps=float(fps),
            macro_block_size=int(cfg.get("video_macro_block_size", 1)),
            quality=int(cfg.get("video_quality", 9)),
        )
    except TypeError:
        imageio.mimsave(video_path, images, fps=float(fps))
    print("=" * 80)
    print("Video created successfully:")
    print(os.path.abspath(video_path))
    print("=" * 80)

def main():
    replay = load_replay()
    cfg = get_cfg(replay)
    raw_dir = cfg.get("raw_frames_dir", "video_frames_raw")
    processed_dir = cfg.get("processed_frames_dir", "video_frames")
    if cfg.get("save_frames", True):
        clean_png_folder(raw_dir)
        clean_png_folder(processed_dir)
    sumo_binary = "sumo-gui" if cfg.get("use_gui", True) else "sumo"
    step_length = str(float(replay.get("dt", replay.get("metadata", {}).get("dt", 1.0))))
    traci.start([sumo_binary, "-c", SUMO_CONFIG, "--start", "--quit-on-end", "--step-length", step_length])
    view_id = get_view_id()
    try:
        traci.gui.setZoom(view_id, float(cfg.get("fixed_zoom", 900.0)))
    except Exception:
        pass
    traci.simulationStep()
    N = int(replay.get("N", replay.get("metadata", {}).get("N", 0)))
    sleep_seconds = float(replay.get("sleep_seconds", 0.0))
    active_poly_ids = []
    for k in range(N + 1):
        replay_step(k, replay)
        update_dynamic_visuals(active_poly_ids, k, replay, cfg)
        update_camera(view_id, k, replay, cfg)
        print(f"Realistic SUMO replay step {k}/{N}: {current_status_text(k, replay)}")
        if cfg.get("save_frames", True):
            frame_path = os.path.join(raw_dir, f"frame_{k:04d}.png")
            try:
                traci.gui.screenshot(view_id, frame_path, width=int(cfg.get("screenshot_width", 1920)), height=int(cfg.get("screenshot_height", 1080)))
            except TypeError:
                try:
                    traci.gui.screenshot(view_id, frame_path, int(cfg.get("screenshot_width", 1920)), int(cfg.get("screenshot_height", 1080)))
                except Exception:
                    traci.gui.screenshot(view_id, frame_path)
            except Exception as e:
                print(f"[warning] could not save screenshot for step {k}: {e}")
        traci.simulationStep()
        if sleep_seconds > 0:
            time.sleep(sleep_seconds)
    remove_polygons(active_poly_ids)
    traci.close()
    if cfg.get("save_frames", True):
        print("=" * 80)
        print("Raw frames saved in:")
        print(os.path.abspath(raw_dir))
        print("=" * 80)
        if cfg.get("postprocess_frames", True):
            final_frames_dir = postprocess_frames(raw_dir, processed_dir, replay, cfg)
        else:
            final_frames_dir = raw_dir
        print("=" * 80)
        print("Final processed frames saved in:")
        print(os.path.abspath(final_frames_dir))
        print("=" * 80)
        if cfg.get("make_video", True):
            video_path = os.path.join(os.getcwd(), cfg.get("video_name", "sumo_replay_realistic.mp4"))
            make_video_from_frames(final_frames_dir, video_path, fps=float(cfg.get("video_fps", 2)), cfg=cfg)

if __name__ == "__main__":
    main()
