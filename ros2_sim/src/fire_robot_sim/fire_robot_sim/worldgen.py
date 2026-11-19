"""SDF for the fires (logs, flame billboards, glow, lights, particle emitters) and the water jet.

Each fire is
  * fire_<id>_pit      static pit: ash, stones, charred logs and twigs, flame + smoke particle emitters, collision;
  * fire_<id>_f<k>_s<j> 4 flame frames x 3 sizes: three crossed billboard planes textured with REAL flame photos
                       (assets/CREDITS.md), an ember glow and a ground halo. They sit hidden under the floor; the fire
                       manager swaps one into place ~8 times a second (frame swap + random turn) and shrinks the fire by
                       moving to a smaller size;
  (each variant carries its own orange point light, so swapping variants also flickers the light)
"""
import math

from fire_robot_sim.core.fire_model import SCALE_LEVELS

N_FRAMES = 4
HIDDEN_Z = -3.0
FLAME_W, FLAME_H = 0.30, 0.36         # metres at base scale
PLANE_ANGLES = (0.0, 60.0, 120.0)


def variant_name(fire_id, frame, level):
    return f"fire_{fire_id}_f{frame}_s{level}"


def emitter_topic(fire_id, kind):
    return f"/fire/{fire_id}/{kind}"


def _vis(name, geom, pose, material, shadows=False):
    return (f'<visual name="{name}"><cast_shadows>{"true" if shadows else "false"}</cast_shadows><pose>{pose}</pose>'
            f"<geometry>{geom}</geometry><material>{material}</material></visual>")


def _color(r, g, b, emissive=None):
    e = f"<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>" if emissive else ""
    return f"<ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0.05 0.05 0.05 1</specular>{e}"


def _cyl(r, l):
    return f"<cylinder><radius>{r}</radius><length>{l}</length></cylinder>"


def pit_model(fire, assets_dir):
    s = float(fire["intensity"]) ** 0.5
    parts = [_vis("ash", _cyl(0.17 * s, 0.003), "0 0 0.0015 0 0 0", _color(0.09, 0.08, 0.07))]
    for i in range(9):                                          # ring of stones
        a = 2 * math.pi * i / 9
        parts.append(_vis(f"stone{i}", f"<sphere><radius>{0.033 * s:.3f}</radius></sphere>",
                          f"{0.165 * s * math.cos(a):.3f} {0.165 * s * math.sin(a):.3f} {0.02 * s:.3f} 0 0 0",
                          _color(0.45 + 0.05 * (i % 3), 0.44, 0.42), shadows=True))
    char = _color(0.13, 0.08, 0.05)
    logs = [(0.0, -0.045, 0.016, 0, 0), (0.0, 0.045, 0.016, 0, 0),            # two logs on the ground (along x)
            (0.0, 0.0, 0.047, 0, 90), (0.01, 0.0, 0.078, 0, 90),            # two across them (along y)
            (-0.01, 0.0, 0.105, 25, 0)]                                      # one leaning on top
    for i, (x, y, z, pitch, yawdeg) in enumerate(logs):
        parts.append(_vis(f"log{i}", _cyl(0.017 * s, 0.17 * s),
                          f"{x * s:.3f} {y * s:.3f} {z * s:.3f} 0 {math.radians(90 - pitch):.3f} {math.radians(yawdeg):.3f}", char, True))
    for i in range(7):                                                     # twigs / debris
        a = 2 * math.pi * i / 7 + 0.4
        parts.append(_vis(f"twig{i}", _cyl(0.004, 0.07), f"{0.07 * s * math.cos(a):.3f} {0.07 * s * math.sin(a):.3f} 0.012 0 1.45 {a:.2f}",
                          _color(0.2, 0.13, 0.08)))
    ramps = f"{assets_dir}/particles"
    emitters = f'''<particle_emitter name="flame" type="ellipsoid"><emitting>true</emitting><pose>0 0 {0.10 * s:.3f} 0 0 0</pose>
          <size>{0.10 * s:.3f} {0.10 * s:.3f} 0.02</size><particle_size>0.012 0.012 0.012</particle_size><rate>24</rate>
          <lifetime>0.9</lifetime><min_velocity>0.20</min_velocity><max_velocity>0.55</max_velocity><scale_rate>0</scale_rate>
          <color_range_image>{ramps}/spark.png</color_range_image><topic>{emitter_topic(fire["id"], "flame")}</topic></particle_emitter>
        <particle_emitter name="smoke" type="ellipsoid"><emitting>true</emitting><pose>0 0 {0.30 * s:.3f} 0 0 0</pose>
          <size>{0.08 * s:.3f} {0.08 * s:.3f} 0.02</size><particle_size>0.04 0.04 0.04</particle_size><rate>16</rate>
          <lifetime>4.5</lifetime><min_velocity>0.12</min_velocity><max_velocity>0.30</max_velocity><scale_rate>0.0</scale_rate>
          <color_range_image>{ramps}/smoke.png</color_range_image><topic>{emitter_topic(fire["id"], "smoke")}</topic></particle_emitter>'''
    return f'''    <model name="fire_{fire["id"]}_pit">
      <static>true</static>
      <pose>{fire["x"]} {fire["y"]} 0 0 0 0</pose>
      <link name="link">
        <collision name="pit"><pose>0 0 0.025 0 0 0</pose><geometry>{_cyl(0.15 * s, 0.05)}</geometry></collision>
        {chr(10).join("        " + p for p in parts)}
        {emitters}
      </link>
    </model>'''


def variant_model(fire, frame, level, assets_dir):
    """One flame look: 3 crossed textured planes + ember glow + ground halo, hidden under the floor until shown."""
    k = float(fire["intensity"]) ** 0.5 * SCALE_LEVELS[level]
    w, h = FLAME_W * k, FLAME_H * k
    tex = f"file://{assets_dir}/flames/flame_{frame + 1}.png"
    pbr = f"<pbr><metal><albedo_map>{tex}</albedo_map><emissive_map>{tex}</emissive_map></metal></pbr>"
    planes = []
    for i, ang in enumerate(PLANE_ANGLES):
        planes.append(_vis(f"flame{i}", f"<box><size>{w:.3f} 0.0004 {h:.3f}</size></box>",
                           f"0 0 {h / 2 + 0.03:.3f} 0 0 {math.radians(ang + 23 * frame):.3f}", pbr))
    glow_tex = f"file://{assets_dir}/flames/glow.png"
    halo = _vis("halo", f"<box><size>{0.75 * k:.3f} {0.75 * k:.3f} 0.0004</size></box>", "0 0 0.006 0 0 0",
                f"<pbr><metal><albedo_map>{glow_tex}</albedo_map><emissive_map>{glow_tex}</emissive_map></metal></pbr>")
    embers = [_vis(f"ember{i}", f"<sphere><radius>{0.022 * k:.3f}</radius></sphere>",
                   f"{0.06 * k * math.cos(2.1 * i + frame):.3f} {0.06 * k * math.sin(2.1 * i + frame):.3f} {0.03 + 0.02 * (i % 2):.3f} 0 0 0",
                   _color(1.0, 0.35, 0.05, emissive=(1.0, 0.30, 0.04))) for i in range(4)]
    jitter = (0.70, 0.88, 0.78, 1.0)[frame]                           # per-frame brightness jitter of the flame light
    glow = (f'<light type="point" name="glow_{fire['id']}_{frame}_{level}"><pose>0 0 {0.20 * k:.3f} 0 0 0</pose><diffuse>{jitter:.2f} {0.5 * jitter:.2f} {0.18 * jitter:.2f} 1</diffuse>'
            f'<specular>0.3 0.15 0.05 1</specular><attenuation><range>2.4</range><constant>0.25</constant><linear>0.2</linear>'
            f'<quadratic>{0.7 / max(k, 0.3):.2f}</quadratic></attenuation><cast_shadows>false</cast_shadows></light>')
    body = "\n        ".join(planes + [halo] + embers + [glow])
    return f'''    <model name="{variant_name(fire["id"], frame, level)}">
      <static>true</static>
      <pose>{fire["x"]} {fire["y"]} {HIDDEN_Z} 0 0 0</pose>
      <link name="link">
        {body}
      </link>
    </model>'''


JET_VARIANTS = ("water_jet_a", "water_jet_b")


def jet_models(range_m=1.1):
    """The water jet: a translucent cone + bright core along +X from the nozzle tip, with droplets near the far end.
    Two variants with different droplet positions are swapped ~10x a second (Gazebo particle emitters cannot be aimed at
    runtime, so the jet is built from visuals). Hidden under the floor until the pump sprays."""
    out = []
    for n, name in enumerate(JET_VARIANTS):
        drops = []
        for i in range(9):
            t = 0.45 + 0.55 * ((i * 0.37 + 0.21 * n) % 1.0)
            off = 0.012 + 0.03 * t
            drops.append(_vis(f"drop{i}", "<sphere><radius>0.007</radius></sphere>",
                              f"{range_m * t:.3f} {off * math.sin(5.1 * i + 2.3 * n):.3f} {off * math.cos(3.7 * i + 1.9 * n) - 0.02 * t:.3f} 0 0 0",
                              "<ambient>0.75 0.88 1 0.7</ambient><diffuse>0.75 0.88 1 0.7</diffuse><emissive>0.35 0.5 0.7 1</emissive>"))
        out.append(f'''    <model name="{name}">
      <static>true</static>
      <pose>0 0 {HIDDEN_Z} 0 0 0</pose>
      <link name="link">
        {_vis("spray", f"<cone><radius>0.045</radius><length>{range_m}</length></cone>", f"{range_m / 2:.3f} 0 0 0 -1.5708 0",
              "<ambient>0.7 0.85 1 0.18</ambient><diffuse>0.7 0.85 1 0.18</diffuse><emissive>0.25 0.4 0.6 1</emissive>")}
        {_vis("core", _cyl(0.004, range_m), f"{range_m / 2:.3f} 0 0 0 1.5708 0",
              "<ambient>0.8 0.9 1 0.6</ambient><diffuse>0.8 0.9 1 0.6</diffuse><emissive>0.4 0.55 0.75 1</emissive>")}
        {chr(10).join("        " + d for d in drops)}
      </link>
    </model>''')
    return "\n".join(out)


def fires_sdf(fires, assets_dir):
    out = []
    for f in fires:
        out.append(pit_model(f, assets_dir))
        for frame in range(N_FRAMES):
            for level in range(len(SCALE_LEVELS)):
                out.append(variant_model(f, frame, level, assets_dir))
    out.append(jet_models())
    return "\n".join(out)
