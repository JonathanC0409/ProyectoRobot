#!/usr/bin/env python3
"""
HTTP server with extensible routing — generic base + MuJoCo example.

SimApp is a minimal HTTP server with decorator-based routing. It knows nothing
about MuJoCo. MujocoSimApp extends it with simulation-specific routes.

Usage:
    # Standalone with built-in MuJoCo routes
    python sim_server.py <scene.xml> [--port 8090] [--width 640] [--height 480]

    # As a library — extend MujocoSimApp
    from sim_server import MujocoSimApp

    app = MujocoSimApp("scene.xml")

    @app.get("/reward")
    def reward(handler, params):
        height = app.data.qpos[2]
        handler.json_response({"reward": float(height)})

    @app.post("/step")  # override built-in
    def custom_step(handler, body):
        n = body.get("n", 1)
        for _ in range(n):
            my_pre_step_hook(app.model, app.data)
            mujoco.mj_step(app.model, app.data)
        handler.json_response({"time": float(app.data.time), "steps": n})

    app.run()

    # Or use SimApp directly for non-MuJoCo servers
    from sim_server import SimApp

    app = SimApp()

    @app.get("/hello")
    def hello(handler, params):
        handler.json_response({"msg": "hello"})

    app.run()
"""

import argparse
import io
import json
import os
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs


# ── SimApp (generic) ─────────────────────────────────────────────────────

class SimApp:
    """
    Minimal HTTP server with decorator-based GET/POST routing.

    No assumptions about what's being served — just routes and handlers.

    Register routes:
        @app.get("/path")
        def handler(handler, params):  ...   # params = parsed query string

        @app.post("/path")
        def handler(handler, body):  ...     # body = parsed JSON dict

    Handlers receive a SimHandler instance with:
        handler.json_response(data, status=200)
        handler.png_response(pixels)
        handler.error_response(msg, status=400)
    """

    def __init__(self):
        self._get_routes = {}
        self._post_routes = {}

    def get(self, path):
        """Decorator to register a GET route."""
        def decorator(fn):
            self._get_routes[path.rstrip("/")] = fn
            return fn
        return decorator

    def post(self, path):
        """Decorator to register a POST route."""
        def decorator(fn):
            self._post_routes[path.rstrip("/")] = fn
            return fn
        return decorator

    def run(self, port=None, host="localhost"):
        """Start the HTTP server."""
        if port is None:
            port = int(os.environ.get("MUJOCO_SIM_PORT", 8090))

        app_ref = self

        class Handler(SimHandler):
            sim_app = app_ref

        print(f"Routes: {len(self._get_routes)} GET, {len(self._post_routes)} POST")
        print(f"Server starting on http://{host}:{port}")

        server = HTTPServer((host, port), Handler)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down.")
            server.server_close()


# ── SimHandler ───────────────────────────────────────────────────────────

class SimHandler(BaseHTTPRequestHandler):
    """HTTP request handler that dispatches to SimApp routes."""

    sim_app = None

    def log_message(self, fmt, *args):
        sys.stderr.write(f"[sim] {args[0]}\n")

    def json_response(self, data, status=200):
        body = json.dumps(data, default=str).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def png_response(self, pixels):
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(pixels).save(buf, format="PNG")
        png = buf.getvalue()
        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", len(png))
        self.end_headers()
        self.wfile.write(png)

    def error_response(self, msg, status=400):
        self.json_response({"error": msg}, status)

    def _read_body(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length))

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        params = parse_qs(parsed.query)
        route = self.sim_app._get_routes.get(path)
        if route:
            route(self, params)
        else:
            self.error_response(f"Unknown GET endpoint: {path}", 404)

    def do_POST(self):
        path = urlparse(self.path).path.rstrip("/")
        try:
            body = self._read_body()
        except json.JSONDecodeError:
            self.error_response("Invalid JSON")
            return
        route = self.sim_app._post_routes.get(path)
        if route:
            route(self, body)
        else:
            self.error_response(f"Unknown POST endpoint: {path}", 404)


# ── MujocoSimApp ─────────────────────────────────────────────────────────

class MujocoSimApp(SimApp):
    """
    SimApp with built-in MuJoCo simulation routes.

    Provides:
        GET  /status, /routes, /screenshot, /sensors, /joints, /bodies, /actuators
        POST /step, /reset, /ctrl, /camera, /solver, /force

    All routes can be overridden with @app.get() / @app.post() decorators.

    Usage:
        # From scene file
        app = MujocoSimApp("scene.xml")

        # From existing model/data (no reload)
        app = MujocoSimApp(model=model, data=data)

        # Add custom routes or override built-ins
        @app.get("/reward")
        def reward(handler, params): ...

        app.run()
    """

    def __init__(self, scene_path=None, model=None, data=None, width=640, height=480):
        super().__init__()
        import mujoco as mj
        self._mj = mj

        self.render_width = width
        self.render_height = height

        if model is not None:
            self.model = model
            self.data = data if data is not None else mj.MjData(model)
        elif scene_path:
            self.model = mj.MjModel.from_xml_path(scene_path)
            self.data = mj.MjData(self.model)
        else:
            self.model = None
            self.data = None

        # Default camera
        self.camera = mj.MjvCamera()
        self.camera.type = mj.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = [0, 0, 1.0]
        self.camera.distance = 3.0
        self.camera.azimuth = 135
        self.camera.elevation = -25

        if self.model is not None:
            mj.mj_forward(self.model, self.data)

        self._register_mujoco_routes()

    def run(self, port=None, host="localhost"):
        m = self.model
        print(f"Model: nq={m.nq} nv={m.nv} nu={m.nu} nbody={m.nbody} nsensor={m.nsensor}")
        super().run(port=port, host=host)

    def _register_mujoco_routes(self):
        import numpy as np
        mj = self._mj
        app = self

        @self.get("/status")
        def status(handler, params):
            handler.json_response({
                "time": float(app.data.time),
                "timestep": float(app.model.opt.timestep),
                "nq": app.model.nq, "nv": app.model.nv, "nu": app.model.nu,
                "nbody": app.model.nbody, "njnt": app.model.njnt,
                "nsensor": app.model.nsensor,
                "solver": int(app.model.opt.solver),
                "integrator": int(app.model.opt.integrator),
            })

        @self.get("/routes")
        def list_routes(handler, params):
            handler.json_response({
                "GET": sorted(app._get_routes.keys()),
                "POST": sorted(app._post_routes.keys()),
            })

        @self.get("/screenshot")
        def screenshot(handler, params):
            cam = mj.MjvCamera()
            cam.type = app.camera.type
            cam.lookat[:] = app.camera.lookat
            cam.distance = app.camera.distance
            cam.azimuth = app.camera.azimuth
            cam.elevation = app.camera.elevation
            cam.fixedcamid = app.camera.fixedcamid
            cam.trackbodyid = app.camera.trackbodyid

            for field in ["azimuth", "elevation", "distance"]:
                if field in params:
                    setattr(cam, field, float(params[field][0]))
            for axis, idx in [("lookat_x", 0), ("lookat_y", 1), ("lookat_z", 2)]:
                if axis in params:
                    cam.lookat[idx] = float(params[axis][0])
            if "camera" in params:
                cam_name = params["camera"][0]
                cam_id = mj.mj_name2id(app.model, mj.mjtObj.mjOBJ_CAMERA, cam_name)
                if cam_id >= 0:
                    cam.type = mj.mjtCamera.mjCAMERA_FIXED
                    cam.fixedcamid = cam_id
                else:
                    handler.error_response(f"Camera '{cam_name}' not found")
                    return

            w = int(params.get("width", [app.render_width])[0])
            h = int(params.get("height", [app.render_height])[0])

            renderer = mj.Renderer(app.model, height=h, width=w)
            renderer.update_scene(app.data, camera=cam)
            pixels = renderer.render()
            renderer.close()
            handler.png_response(pixels)

        @self.get("/sensors")
        def sensors(handler, params):
            result = {}
            for i in range(app.model.nsensor):
                name = app.model.sensor(i).name
                result[name] = app.data.sensor(name).data.tolist()
            handler.json_response({"time": float(app.data.time), "sensors": result})

        @self.get("/joints")
        def joints(handler, params):
            jts = {}
            for i in range(app.model.njnt):
                name = app.model.joint(i).name
                jts[name] = {
                    "qpos": app.data.joint(name).qpos.tolist(),
                    "qvel": app.data.joint(name).qvel.tolist(),
                }
            handler.json_response({"time": float(app.data.time), "joints": jts})

        @self.get("/bodies")
        def bodies(handler, params):
            bds = {}
            for i in range(app.model.nbody):
                name = app.model.body(i).name
                bds[name] = {
                    "pos": app.data.xpos[i].tolist(),
                    "quat": app.data.xquat[i].tolist(),
                }
            handler.json_response({"time": float(app.data.time), "bodies": bds})

        @self.get("/actuators")
        def actuators(handler, params):
            acts = []
            for i in range(app.model.nu):
                acts.append({
                    "id": i,
                    "name": app.model.actuator(i).name,
                    "ctrl": float(app.data.ctrl[i]),
                })
            handler.json_response({"actuators": acts})

        @self.post("/step")
        def step(handler, body):
            n = body.get("n", 1)
            for _ in range(n):
                mj.mj_step(app.model, app.data)
            handler.json_response({"time": float(app.data.time), "steps": n})

        @self.post("/reset")
        def reset(handler, body):
            keyframe = body.get("keyframe")
            if keyframe is not None:
                mj.mj_resetDataKeyframe(app.model, app.data, keyframe)
            else:
                mj.mj_resetData(app.model, app.data)
            mj.mj_forward(app.model, app.data)
            handler.json_response({"time": float(app.data.time), "status": "reset"})

        @self.post("/ctrl")
        def ctrl(handler, body):
            if "ctrl" in body:
                app.data.ctrl[:] = np.array(body["ctrl"], dtype=np.float64)
            elif "name" in body and "value" in body:
                act_id = mj.mj_name2id(app.model, mj.mjtObj.mjOBJ_ACTUATOR, body["name"])
                if act_id < 0:
                    handler.error_response(f"Actuator '{body['name']}' not found")
                    return
                app.data.ctrl[act_id] = float(body["value"])
            elif "values" in body:
                for name, val in body["values"].items():
                    act_id = mj.mj_name2id(app.model, mj.mjtObj.mjOBJ_ACTUATOR, name)
                    if act_id < 0:
                        handler.error_response(f"Actuator '{name}' not found")
                        return
                    app.data.ctrl[act_id] = float(val)
            handler.json_response({"ctrl": app.data.ctrl.tolist()})

        @self.post("/camera")
        def camera(handler, body):
            for field in ["azimuth", "elevation", "distance"]:
                if field in body:
                    setattr(app.camera, field, float(body[field]))
            if "lookat" in body:
                app.camera.lookat[:] = body["lookat"]
            if "type" in body:
                app.camera.type = int(body["type"])
            if "trackbodyid" in body:
                app.camera.trackbodyid = int(body["trackbodyid"])
            if "fixedcamid" in body:
                app.camera.fixedcamid = int(body["fixedcamid"])
            if "camera_name" in body:
                cam_id = mj.mj_name2id(app.model, mj.mjtObj.mjOBJ_CAMERA, body["camera_name"])
                if cam_id >= 0:
                    app.camera.type = mj.mjtCamera.mjCAMERA_FIXED
                    app.camera.fixedcamid = cam_id
            handler.json_response({
                "azimuth": app.camera.azimuth, "elevation": app.camera.elevation,
                "distance": app.camera.distance, "lookat": app.camera.lookat.tolist(),
            })

        @self.post("/solver")
        def solver(handler, body):
            for field in ["solver", "integrator", "iterations"]:
                if field in body:
                    setattr(app.model.opt, field, int(body[field]))
            for field in ["tolerance", "timestep"]:
                if field in body:
                    setattr(app.model.opt, field, float(body[field]))
            if "gravity" in body:
                app.model.opt.gravity[:] = body["gravity"]
            handler.json_response({
                "solver": int(app.model.opt.solver),
                "integrator": int(app.model.opt.integrator),
                "iterations": app.model.opt.iterations,
                "tolerance": app.model.opt.tolerance,
                "timestep": float(app.model.opt.timestep),
                "gravity": app.model.opt.gravity.tolist(),
            })

        @self.post("/force")
        def force(handler, body):
            body_name = body.get("body")
            if not body_name:
                handler.error_response("Missing 'body' field")
                return
            body_id = mj.mj_name2id(app.model, mj.mjtObj.mjOBJ_BODY, body_name)
            if body_id < 0:
                handler.error_response(f"Body '{body_name}' not found")
                return
            f = body.get("force", [0, 0, 0, 0, 0, 0])
            app.data.xfrc_applied[body_id] = f
            handler.json_response({"body": body_name, "force": f})


# ── CLI entry point ──────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="MuJoCo Simulation HTTP Server")
    parser.add_argument("scene", help="Path to MuJoCo XML scene file")
    parser.add_argument("--port", type=int, default=int(os.environ.get("MUJOCO_SIM_PORT", 8090)))
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    print(f"Loading model: {args.scene}")
    app = MujocoSimApp(args.scene, width=args.width, height=args.height)
    app.run(port=args.port)


if __name__ == "__main__":
    main()
