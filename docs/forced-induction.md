# Forced induction (supercharger and turbocharger)

A `Compressor` part feeds one intake. Its only coupling into the gas model is the
pressure and temperature of the air the intake draws from (`Intake::setSource`), so the
plenum, runners, chambers, exhaust and audio all respond to boost on their own.

## Script

```
supercharger blower(intake: intake, max_boost: 9.0 * units.psi, boost_speed: 4500 * units.rpm,
                    drive_ratio: 2.6, intercooler_efficiency: 0.4, whine_volume: 1.2)
engine.add_supercharger(blower)

turbocharger turbo(intake: intake, exhaust_system: exhaust0, max_boost: 14.0 * units.psi,
                   turbine_inertia: 0.00002, max_speed: 150000 * units.rpm,
                   exhaust_restriction: 0.55, blow_off_volume: 1.0, whine_volume: 0.6)
engine.add_turbocharger(turbo)
```

Examples: `assets/engines/forced-induction/`.

* **Supercharger**: boost = `max_boost * (crank/boost_speed)^2 * throttle`, capped at `max_boost`
  (the bypass). It loads the crankshaft with `parasitic_torque + compression power / crank speed`.
* **Turbocharger**: turbine power comes from the exhaust collector (pressure above ambient times
  molar flow); shaft speed integrates turbine minus compressor power over `turbine_inertia`, which
  makes the lag. At `max_speed` the wastegate opens. The turbine adds back-pressure through
  `exhaust_restriction`. Closing the throttle under boost vents the charge (blow-off).
* **Audio**: `whine_volume` / `whine_order` (shaft tone) and `blow_off_volume` (hiss) are mixed into
  the exhaust synthesizer channel. Shaft tones above the synthesizer input rate alias, so keep the
  order low.
* The manifold pressure gauge now reads boost (range -30 to +30).

## Headless check

`boost-probe <main.mr> [rpm ...]` holds the dyno at each speed at full throttle and prints boost,
shaft speed, charge temperature, dyno torque and drive load. See `assets/probe_*.mr`.

## Notes

* `units.psi` in `es/constants/units.mr` used mass-pounds; it now uses pound-force.
* Building with current Boost (1.85+): Piranha calls the removed `path::is_complete()`. Apply
  `patches/piranha-boost-is-absolute.patch` inside `dependencies/submodules/piranha`.
* Upstream source ends at v0.1.11a and has no Wankel engine, so boosted rotaries are not possible here.
