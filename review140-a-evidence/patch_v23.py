import sys
S = "/private/tmp/claude-501/-Users-r-git-pulseengine-relay/f207d8bc-6a92-4057-9e33-a06782c12f0c/scratchpad"
which = sys.argv[1]
f = S + f"/rv-{which}-tree/crates/falcon-core/plain/src/lib.rs"
s = open(f).read()
a = "        self.filter_dts[self.filter_dt_head] = dt;\n"
assert s.count(a) == 1
if which == "v2":
    ins = "        if resynced_rate(self.filter_hz, dt).is_some() { self.notch = relay_notch::HarmonicNotchBank::new(self.filter_hz); }\n"
elif which == "v3":
    ins = "        if resynced_rate(self.filter_hz, dt).is_some() { self.gyro_lpf = GyroLpf::new(60.0, self.filter_hz); }\n"
s = s.replace(a, ins + a)
open(f, "w").write(s)
print("patched", which)
