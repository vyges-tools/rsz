// SPDX-License-Identifier: Apache-2.0
//! The call-sequence trace: one `tag|field|…` line per decision, in the order the repair makes
//! them, so a run can be lined up against another one stage at a time.
//!
//! Numbers are written as `{:.9g}` is in C++ `fmt`: nine significant digits, trailing zeros and a
//! trailing point dropped, scientific form when the decimal exponent is below −4 or at least 9,
//! the exponent signed and at least two digits. An `f32` is widened to `f64` first, exactly.

/// `x` as `fmt`'s `{:.9g}`.
pub fn g9(x: f64) -> String {
    g(x, 9)
}

/// `x` as `fmt`'s `{:.<prec>g}`.
pub fn g(x: f64, prec: usize) -> String {
    if x.is_nan() {
        return "nan".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf".into() } else { "-inf".into() };
    }
    if x == 0.0 {
        return if x.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    let prec = prec.max(1);
    // Round to `prec` significant digits first; the exponent is the rounded value's.
    let sci = format!("{:.*e}", prec - 1, x);
    let (mant, exp) = sci.split_once('e').expect("scientific form");
    let exp: i32 = exp.parse().expect("exponent");
    if exp < -4 || exp >= prec as i32 {
        let mant = trim_zeros(mant);
        let sign = if exp < 0 { '-' } else { '+' };
        format!("{mant}e{sign}{:02}", exp.abs())
    } else {
        let decimals = (prec as i32 - 1 - exp).max(0) as usize;
        trim_zeros(&format!("{:.*}", decimals, x)).to_string()
    }
}

fn trim_zeros(s: &str) -> &str {
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.')
    } else {
        s
    }
}

/// The trace being written: lines collected in order.
#[derive(Debug, Default, Clone)]
pub struct Trace {
    pub lines: Vec<String>,
    /// The tags of every stage that RAN, whether or not it wrote a line: a comparison must
    /// score a stage that wrote nothing as nothing, not skip it.
    pub tags: std::collections::BTreeSet<&'static str>,
}

impl Trace {
    /// A stage ran; these are the tags it writes.
    pub fn ran(&mut self, tags: &[&'static str]) {
        self.tags.extend(tags.iter().copied());
    }
    pub fn push(&mut self, line: String) {
        self.lines.push(line);
    }
    pub fn text(&self) -> String {
        let mut s = self.lines.join("\n");
        if !s.is_empty() {
            s.push('\n');
        }
        s
    }
}

/// C's `%a` of a double, as glibc prints it: `0x1.<hex, trailing zeros dropped>p<exp>`, `0x0p+0`
/// for zero, `inf` / `-inf` — the instrumented reference prints its floats so (`VYGV` lines).
pub fn c_hex(v: f64) -> String {
    if v.is_infinite() {
        return if v < 0.0 { "-inf".into() } else { "inf".into() };
    }
    if v.is_nan() {
        return "nan".into();
    }
    let sign = if v.is_sign_negative() { "-" } else { "" };
    if v == 0.0 {
        return format!("{sign}0x0p+0");
    }
    let bits = v.to_bits();
    let mut exp = ((bits >> 52) & 0x7ff) as i64;
    let mant = bits & ((1u64 << 52) - 1);
    let lead = if exp == 0 {
        // Subnormal: glibc prints 0x0.<mant>p-1022.
        exp = -1022;
        0
    } else {
        exp -= 1023;
        1
    };
    let mut digits = format!("{mant:013x}");
    while digits.ends_with('0') {
        digits.pop();
    }
    let frac = if digits.is_empty() { String::new() } else { format!(".{digits}") };
    let esign = if exp < 0 { "-" } else { "+" };
    format!("{sign}0x{lead}{frac}p{esign}{}", exp.abs())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule: fmt `{:.9g}` — the forms seen in a reference trace.
    #[test]
    fn nine_significant_digits_as_fmt_writes_them() {
        assert_eq!(g9(f64::from(0.932_027_7_f32)), "0.932027698");
        assert_eq!(g9(f64::from(76.778_694_f32)), "76.7786942");
        assert_eq!(g9(f64::from(1.298_828_1e-13_f32)), "1.29882812e-13");
        assert_eq!(g9(f64::from(1e30_f32)), "1.00000002e+30");
        assert_eq!(g9(0.0), "0");
        assert_eq!(g9(1.0), "1");
        assert_eq!(g9(1_600_000.0), "1600000");
        assert_eq!(g9(123_456_789_012.0), "1.23456789e+11");
        assert_eq!(g9(0.0001), "0.0001");
        assert_eq!(g9(0.00001), "1e-05");
    }
}
