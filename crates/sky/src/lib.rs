//! # gta-sky
//!
//! Bounded context: **sky, sun and weather**.
//!
//! The atmosphere model: where the sun is, what colour the light is, how thick the haze
//! is and how dark the night has become. CPU-side only — it produces the numbers `render`
//! uploads as a uniform, and the `night` factor that switches city windows and street
//! lights on.
//!
//! Time is a wall-clock hour on a 24 h clock (`0.0` = midnight, `12.0` = noon). The sun
//! runs a tilted circle: due east at sunrise, high but leaning at noon, down in the west.
//! Deliberately not an ephemeris — it only has to look right.

use gta_math::{clamp, smoothstep, Vec3, TAU};

/// Knobs for the atmosphere.
#[derive(Clone, Debug)]
pub struct SkyParams {
    /// Seconds of simulated time for one full 24 h cycle.
    pub day_length: f32,
    /// Clock hour at simulation time zero.
    pub start_hour: f32,
    /// Lean of the sun path away from the zenith at noon (0 = overhead, 1 = low).
    pub noon_lean: f32,
    /// Standing haze 0..1 — humid cities read milier.
    pub haze: f32,
    /// Overcast 0..1 — flattens light, kills stars.
    pub cloud: f32,
    /// How strongly the city's own lights lift the night ambient (0..1).
    pub night_lights: f32,
}

impl Default for SkyParams {
    fn default() -> Self {
        SkyParams {
            day_length: 240.0,
            start_hour: 8.5,
            noon_lean: 0.35,
            haze: 0.35,
            cloud: 0.15,
            night_lights: 1.0,
        }
    }
}

/// One frame's worth of atmosphere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sky {
    /// Unit vector pointing *towards* the sun.
    pub sun_dir: Vec3,
    /// Direct sunlight colour, including horizon reddening.
    pub sun_color: Vec3,
    /// Direct-light strength multiplier.
    pub sun_intensity: f32,
    /// Downwelling sky light.
    pub ambient_sky: Vec3,
    /// Light bounced back up off the ground.
    pub ambient_ground: Vec3,
    /// Horizon colour — also the fog colour, so geometry dissolves into the sky.
    pub horizon: Vec3,
    /// Zenith colour.
    pub zenith: Vec3,
    /// Exponential-squared fog density (1/m).
    pub fog_density: f32,
    /// 0 = broad day, 1 = full night. Drives window glow and street lights.
    pub night: f32,
    /// 0..1 star visibility.
    pub stars: f32,
    /// Tone-map exposure.
    pub exposure: f32,
    /// Wrapped clock hour 0..24, for the HUD.
    pub hour: f32,
}

impl Sky {
    /// Atmosphere at simulation time `t` seconds.
    #[inline]
    pub fn at_time(t: f32, p: &SkyParams) -> Sky {
        Sky::at(p.start_hour + t / p.day_length.max(1e-3) * 24.0, p)
    }

    /// Atmosphere at clock `hour` (any real number; it wraps).
    pub fn at(hour: f32, p: &SkyParams) -> Sky {
        let hour = hour.rem_euclid(24.0);
        // Solar phase: -PI/2 at midnight, 0 at sunrise, PI/2 at noon, PI at sunset.
        let theta = (hour - 6.0) / 24.0 * TAU;
        let sun_dir = Vec3::new(theta.cos(), theta.sin(), p.noon_lean).normalize();
        let elev = sun_dir.y;

        let night = smoothstep(clamp(0.18 - 3.0 * elev, 0.0, 1.0));
        let day = 1.0 - night;

        // Direct beam: white overhead, orange when grazing, blood-red at the last moment.
        let grazing = 1.0 - clamp(elev, 0.0, 1.0);
        let mut sun = Vec3::new(1.0, 0.97, 0.88).lerp(Vec3::new(1.0, 0.58, 0.3), smoothstep(grazing));
        sun = sun.lerp(Vec3::new(1.0, 0.32, 0.13), smoothstep(clamp((grazing - 0.85) * 6.0, 0.0, 1.0)));
        sun = sun.lerp(Vec3::new(0.8, 0.81, 0.85), p.cloud * day * 0.75);
        let sun_intensity = (0.12 + 1.6 * clamp(elev, 0.0, 1.0)) * (1.0 - p.cloud * 0.6);

        // Sky gradient, blended from three key palettes.
        let (d_zen, d_hor) = (Vec3::new(0.16, 0.42, 0.78), Vec3::new(0.63, 0.72, 0.84));
        let (k_zen, k_hor) = (Vec3::new(0.12, 0.14, 0.32), Vec3::new(0.95, 0.46, 0.25));
        let (n_zen, n_hor) = (Vec3::new(0.015, 0.028, 0.07), Vec3::new(0.05, 0.06, 0.11));
        // Twilight weight peaks while the sun sits on the horizon.
        let twilight = smoothstep(clamp(1.0 - elev.abs() * 4.5, 0.0, 1.0)) * day;

        let mut zenith = d_zen.lerp(n_zen, night).lerp(k_zen, twilight * 0.7);
        let mut horizon = d_hor.lerp(n_hor, night).lerp(k_hor, twilight);

        let haze_tint = Vec3::new(0.74, 0.78, 0.82);
        horizon = horizon.lerp(haze_tint, p.haze * 0.35 * day);
        zenith = zenith.lerp(haze_tint, p.haze * 0.18 * day);
        zenith = zenith.lerp(Vec3::new(0.42, 0.44, 0.47), p.cloud * day * 0.8);
        horizon = horizon.lerp(Vec3::new(0.5, 0.52, 0.54), p.cloud * day * 0.8);

        // Ambient never reaches zero: a city glows after dark.
        let ambient_sky = Vec3::new(0.34, 0.42, 0.58) * (0.1 + 0.95 * clamp(elev, 0.0, 1.0))
            + Vec3::new(0.3, 0.2, 0.1) * (0.25 * p.night_lights * night);
        let ambient_ground = Vec3::new(0.26, 0.24, 0.21) * (0.08 + 0.8 * clamp(elev, 0.0, 1.0))
            + Vec3::new(0.07, 0.055, 0.045) * (0.5 * p.night_lights * night);

        Sky {
            sun_dir,
            sun_color: sun,
            sun_intensity,
            ambient_sky,
            ambient_ground,
            horizon,
            zenith,
            fog_density: 0.0013 + 0.0026 * p.haze + 0.0016 * night + 0.0012 * p.cloud,
            night,
            stars: clamp((night - 0.55) * 2.4, 0.0, 1.0) * (1.0 - p.cloud),
            exposure: 1.0 + 0.45 * night,
            hour,
        }
    }

    /// Broad daylight.
    #[inline]
    pub fn noon(p: &SkyParams) -> Sky {
        Sky::at(12.0, p)
    }

    /// Dead of night.
    #[inline]
    pub fn midnight(p: &SkyParams) -> Sky {
        Sky::at(0.0, p)
    }

    /// Should street lights and lit windows be on?
    #[inline]
    pub fn lights_on(&self) -> bool {
        self.night > 0.3
    }

    /// Sun altitude in degrees above the horizon (negative when below).
    #[inline]
    pub fn altitude_deg(&self) -> f32 {
        clamp(self.sun_dir.y, -1.0, 1.0).asin().to_degrees()
    }

    /// `true` in the amber band around sunrise/sunset.
    #[inline]
    pub fn is_twilight(&self) -> bool {
        self.sun_dir.y.abs() < 0.18
    }

    /// One-line summary for logs and the HUD.
    pub fn summary(&self) -> String {
        format!(
            "{:05.1}h alt {:+6.1}deg night {:.2} sun {:.2} fog {:.5}",
            self.hour,
            self.altitude_deg(),
            self.night,
            self.sun_intensity,
            self.fog_density
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn sun_is_up_at_noon_and_down_at_midnight() {
        let p = SkyParams::default();
        assert!(Sky::noon(&p).sun_dir.y > 0.6);
        assert!(Sky::midnight(&p).sun_dir.y < -0.3);
    }

    #[test]
    fn night_flag_tracks_the_sun() {
        let p = SkyParams::default();
        assert!(!Sky::noon(&p).lights_on());
        assert!(Sky::midnight(&p).lights_on());
        assert_eq!(Sky::noon(&p).night, 0.0);
        assert!(Sky::midnight(&p).night > 0.9);
    }

    fn lum(v: Vec3) -> f32 {
        v.x + v.y + v.z
    }

    #[test]
    fn horizon_is_brighter_than_the_zenith() {
        // Both must stay in [0,1] and be brighter at the horizon (aerosols).
        let p = SkyParams::default();
        for h in [0.0, 6.0, 9.0, 12.0, 18.0, 21.0] {
            let s = Sky::at(h, &p);
            for v in [s.zenith.x, s.zenith.y, s.zenith.z, s.horizon.x, s.horizon.y, s.horizon.z] {
                assert!(v >= 0.0 && v <= 1.5, "sky channel out of range at {h}h");
            }
            let lz: f32 = lum(s.zenith);
            let lh: f32 = lum(s.horizon);
            assert!(lh > lz * 0.9, "horizon {lh} dimmer than zenith {lz} at {h}h");
        }
    }

    #[test]
    fn night_is_darker_than_day() {
        let p = SkyParams::default();
        let d = Sky::noon(&p);
        let n = Sky::midnight(&p);
        let bright_of = |s: &Sky| lum(s.ambient_sky) + s.sun_intensity;
        assert!(bright_of(&d) > 4.0 * bright_of(&n));
    }

    #[test]
    fn sunrise_is_in_the_east_and_sunset_in_the_west() {
        let p = SkyParams::default();
        assert!(Sky::at(6.0, &p).sun_dir.x > 0.9, "sunrise +x");
        assert!(Sky::at(18.0, &p).sun_dir.x < -0.9, "sunset -x");
    }

    #[test]
    fn hours_wrap() {
        let p = SkyParams::default();
        let a = Sky::at(13.0, &p);
        let b = Sky::at(37.0, &p);
        assert!(near(a.hour, b.hour, 1e-3));
        assert!(near(a.sun_dir.y, b.sun_dir.y, 1e-5));
        let c = Sky::at(-11.0, &p);
        assert!(near(c.hour, 13.0, 1e-3));
    }

    #[test]
    fn sun_direction_is_normalised() {
        let p = SkyParams::default();
        for i in 0..48 {
            let s = Sky::at(i as f32 * 0.5, &p);
            assert!((s.sun_dir.length() - 1.0).abs() < 1e-5);
        }
    }

    #[test]
    fn time_advances_the_clock() {
        let p = SkyParams { day_length: 100.0, start_hour: 0.0, ..Default::default() };
        let s = Sky::at_time(25.0, &p);
        assert!(near(s.hour, 6.0, 1e-3), "hour {}", s.hour);
        // A full cycle returns to the same place.
        let t = Sky::at_time(100.0, &p);
        assert!(near(t.hour, 0.0, 1e-3) || near(t.hour, 24.0, 1e-3));
    }

    #[test]
    fn haze_and_cloud_thicken_the_fog() {
        let clear = SkyParams { haze: 0.0, cloud: 0.0, ..Default::default() };
        let murky = SkyParams { haze: 1.0, cloud: 0.9, ..Default::default() };
        assert!(Sky::noon(&clear).fog_density < Sky::noon(&murky).fog_density);
    }

    #[test]
    fn cloud_kills_the_stars_and_flattens_the_beam() {
        let p = SkyParams { cloud: 1.0, ..Default::default() };
        let s = Sky::midnight(&p);
        assert_eq!(s.stars, 0.0);
        let clear = SkyParams { cloud: 0.0, ..Default::default() };
        assert!(Sky::midnight(&clear).stars > 0.5);
        assert!(Sky::noon(&p).sun_intensity < Sky::noon(&clear).sun_intensity);
    }

    #[test]
    fn city_lights_lift_the_night_ambient() {
        let dark = SkyParams { night_lights: 0.0, ..Default::default() };
        let bright = SkyParams { night_lights: 1.0, ..Default::default() };
        let a = lum(Sky::midnight(&dark).ambient_sky);
        let b = lum(Sky::midnight(&bright).ambient_sky);
        assert!(b > a);
    }

    #[test]
    fn sunset_goes_through_twilight() {
        let p = SkyParams::default();
        let s = Sky::at(18.4, &p);
        assert!(s.is_twilight(), "alt {}", s.altitude_deg());
        // The beam should be orange/red: red channel dominant.
        assert!(s.sun_color.x > s.sun_color.z, "{:?}", s.sun_color);
    }

    #[test]
    fn summary_mentions_the_hour() {
        let s = Sky::at(9.5, &SkyParams::default());
        assert!(s.summary().contains("09.5h"), "{}", s.summary());
    }
}
