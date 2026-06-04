#!/usr/bin/env python3
"""Generate a realism gz world from falcon-quad.sdf + a small overlay.

Replaces six near-duplicate ~18 KB SDF files (falcon-quad-{wind,drag,imubias,
gnss,baro,battery}.sdf) with one base + thin overlays — so an airframe edit is
made once, not 6×. Each overlay adds only the realism layer's elements (the gz
Harmonic systems / sensor-noise the matching v1.16–v1.21 release exercises).

Usage:
  gen-realism-world.py <layer> [out.sdf]      layer ∈ wind drag imubias gnss
                                              baro battery all
  (default out: /tmp/falcon-quad-<layer>.sdf)

The verifiable proof of each layer is the no_std SimBackend test in
crates/falcon-core; this world is the gz-side physical-fidelity artifact
(bench-only, like the other gz runs).
"""
import sys, os

BASE = os.path.join(os.path.dirname(__file__), "falcon-quad.sdf")


def with_world_child(s, xml):
    i = s.rfind("</world>")
    return s[:i] + xml + "\n  " + s[i:]


def after_sensors_system(s, xml):
    i = s.index("</plugin>", s.index('filename="gz-sim-sensors-system"')) + len("</plugin>")
    return s[:i] + "\n" + xml + s[i:]


def in_base_link(s, xml):  # before base_link's </link>
    i = s.index("</link>", s.index('name="mag_sensor"'))
    return s[:i] + xml + "      " + s[i:]


def wind(s):
    blk = '''    <wind><linear_velocity>5 0 0</linear_velocity></wind>
    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>1.0</force_approximation_scaling_factor>
      <horizontal><magnitude><time_for_rise>10</time_for_rise>
        <sin><amplitude_percent>0.1</amplitude_percent><period>20</period></sin>
        <noise type="gaussian"><mean>0</mean><stddev>0.05</stddev></noise></magnitude>
        <direction><time_for_rise>20</time_for_rise>
        <sin><amplitude>10</amplitude><period>15</period></sin>
        <noise type="gaussian"><mean>0</mean><stddev>0.03</stddev></noise></direction></horizontal>
      <vertical><noise type="gaussian"><mean>0</mean><stddev>0.03</stddev></noise></vertical>
    </plugin>
'''
    s = after_sensors_system(s, blk)
    bl = s.index('<link name="base_link">')
    ie = s.index(">", bl) + 1
    return s[:ie] + "\n        <enable_wind>true</enable_wind>" + s[ie:]


def drag(s):
    return after_sensors_system(s, '''    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <link_name>base_link</link_name><air_density>1.2041</air_density>
      <cla>0.1</cla><cda>0.6</cda><cla_stall>0.0</cla_stall><cda_stall>1.0</cda_stall>
      <alpha_stall>1.5708</alpha_stall><area>0.1</area><cp>0 0 0</cp>
      <upward>0 0 1</upward><forward>1 0 0</forward>
    </plugin>
''')


def imubias(s):
    a = s.index("<angular_velocity>"); b = s.index("</angular_velocity>") + len("</angular_velocity>")
    bias = ("<bias_mean>7.5e-6</bias_mean><bias_stddev>8e-7</bias_stddev>"
            "<dynamic_bias_stddev>2e-5</dynamic_bias_stddev>"
            "<dynamic_bias_correlation_time>400</dynamic_bias_correlation_time>")
    return s[:a] + s[a:b].replace("</stddev></noise>", "</stddev>" + bias + "</noise>") + s[b:]


def gnss(s):
    nav = '''          <navsat>
            <position_sensing>
              <horizontal><noise type="gaussian"><mean>0</mean><stddev>0.3</stddev></noise></horizontal>
              <vertical><noise type="gaussian"><mean>0</mean><stddev>0.6</stddev></noise></vertical>
            </position_sensing>
            <velocity_sensing>
              <horizontal><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></horizontal>
              <vertical><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></vertical>
            </velocity_sensing>
          </navsat>
'''
    i = s.index("<sensor name=\"navsat_sensor\""); close = s.index("</sensor>", i)
    return s[:close] + nav + "        " + s[close:]


def baro(s):
    s = in_base_link(s, '''        <sensor name="air_pressure_sensor" type="air_pressure">
          <always_on>1</always_on><update_rate>50</update_rate>
          <air_pressure><pressure><noise type="gaussian"><mean>0</mean><stddev>3</stddev></noise></pressure></air_pressure>
        </sensor>
''')
    j = s.index(">", s.index('<world name="falcon">')) + 1
    atm = '\n    <atmosphere type="adiabatic"><temperature>288.15</temperature><pressure>101325</pressure><temperature_gradient>-0.0065</temperature_gradient></atmosphere>\n'
    return s[:j] + atm + s[j:]


def battery(s):
    bl = s.index('<link name="base_link">')
    ie = s.index(">", bl) + 1
    return s[:ie] + '''
        <battery name="linear_battery"><voltage>16.8</voltage></battery>''' + s[ie:].replace(
        "</link>", '''      <plugin filename="gz-sim-linearbatteryplugin-system" name="gz::sim::systems::LinearBatteryPlugin">
        <battery_name>linear_battery</battery_name><voltage>16.8</voltage>
        <open_circuit_voltage_constant_coef>16.8</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-4.2</open_circuit_voltage_linear_coef>
        <initial_charge>5.0</initial_charge><capacity>5.0</capacity>
        <resistance>0.07</resistance><smooth_current_tau>2.0</smooth_current_tau>
        <power_load>40</power_load><start_draining>true</start_draining>
      </plugin>
      </link>''', 1)


LAYERS = {"wind": wind, "drag": drag, "imubias": imubias, "gnss": gnss, "baro": baro, "battery": battery}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in list(LAYERS) + ["all"]:
        sys.exit(f"usage: {sys.argv[0]} <{'|'.join(list(LAYERS)+['all'])}> [out.sdf]")
    layer = sys.argv[1]
    s = open(BASE).read()
    for fn in (LAYERS.values() if layer == "all" else [LAYERS[layer]]):
        s = fn(s)
    out = sys.argv[2] if len(sys.argv) > 2 else f"/tmp/falcon-quad-{layer}.sdf"
    open(out, "w").write(s)
    print(out)


if __name__ == "__main__":
    main()
