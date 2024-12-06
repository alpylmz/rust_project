use std::sync::Mutex;

// Define modes
#[derive(Debug, PartialEq, Eq, Clone)] // Added Clone here
pub enum Mode {
    DEBUG,
    ANALYSIS,
}

// Global mode (thread-safe using Mutex)
lazy_static::lazy_static! {
    static ref GLOBAL_MODE: Mutex<Mode> = Mutex::new(Mode::DEBUG);
    pub static ref TEMP_COUNTER: Mutex<usize> = Mutex::new(0); // Counter for temporary names
}


// Set the mode
pub fn set_mode(mode: Mode) {
    let mut global_mode = GLOBAL_MODE.lock().unwrap();
    *global_mode = mode;
}

// Get the current mode
pub fn get_mode() -> Mode {
    let global_mode = GLOBAL_MODE.lock().unwrap();
    global_mode.clone() // Now works because Mode implements Clone
}
