#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Time {
    /// [sec]
    sec: u64,
    /// [nsec]
    nanosec: u64,
}

impl Time {
    pub fn new(sec: u64, nanosec: u64) -> Self {
        Self { sec, nanosec }
    }

    pub fn sec(&self) -> u64 {
        self.sec
    }

    pub fn nanosec(&self) -> u64 {
        self.nanosec
    }

    pub fn sec_as_f64(&self) -> f64 {
        self.sec as f64 + self.nanosec as f64 * 1e-9
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Header {
    /// [sec]
    stamp: Time,
    /// frame_id
    frame_id: String,
}

impl Header {
    pub fn new(stamp: Time, frame_id: String) -> Self {
        Self { stamp, frame_id }
    }

    pub fn stamp(&self) -> &Time {
        &self.stamp
    }

    pub fn frame_id(&self) -> &str {
        &self.frame_id
    }
}
