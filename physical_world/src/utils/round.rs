pub trait Round {
    fn round_digits(self, digits: u32) -> Self;
}

impl Round for f64 {
    fn round_digits(self, digits: u32) -> Self {
        let n = 10_u32.pow(digits) as f64;
        (self * n).round() / n
    }
}
