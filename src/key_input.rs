#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum KeyAction {
    Forward = 1,
    Backward = 2,
    Left = 3,
    Right = 4,
    Stop = 5,
    Quit = 6,
}

impl KeyAction {
    pub fn from_byte(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Forward),
            2 => Some(Self::Backward),
            3 => Some(Self::Left),
            4 => Some(Self::Right),
            5 => Some(Self::Stop),
            6 => Some(Self::Quit),
            _ => None,
        }
    }
}

#[derive(Default)]
pub struct EscapeParser {
    state: u8,
}

impl EscapeParser {
    pub fn push(&mut self, byte: u8) -> Option<KeyAction> {
        match self.state {
            0 => match byte {
                0x1b => {
                    self.state = 1;
                    None
                }
                b' ' => Some(KeyAction::Stop),
                b'q' | b'Q' | 0x03 => Some(KeyAction::Quit),
                _ => None,
            },
            1 => {
                self.state = if byte == b'[' || byte == b'O' { 2 } else { 0 };
                None
            }
            _ => {
                self.state = 0;
                match byte {
                    b'A' => Some(KeyAction::Forward),
                    b'B' => Some(KeyAction::Backward),
                    b'D' => Some(KeyAction::Left),
                    b'C' => Some(KeyAction::Right),
                    _ => None,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_arrows_and_stop() {
        let mut parser = EscapeParser::default();
        assert_eq!(parser.push(0x1b), None);
        assert_eq!(parser.push(b'['), None);
        assert_eq!(parser.push(b'A'), Some(KeyAction::Forward));
        assert_eq!(parser.push(b' '), Some(KeyAction::Stop));
        assert_eq!(parser.push(0x1b), None);
        assert_eq!(parser.push(b'O'), None);
        assert_eq!(parser.push(b'D'), Some(KeyAction::Left));
    }

    #[test]
    fn parses_all_directions_and_quit_keys() {
        let mut parser = EscapeParser::default();
        for (sequence, action) in [
            (&b"\x1b[B"[..], KeyAction::Backward),
            (&b"\x1b[C"[..], KeyAction::Right),
            (&b"\x1bOA"[..], KeyAction::Forward),
        ] {
            let mut result = None;
            for &byte in sequence {
                result = parser.push(byte);
            }
            assert_eq!(result, Some(action));
        }
        assert_eq!(parser.push(b'Q'), Some(KeyAction::Quit));
        assert_eq!(parser.push(0x03), Some(KeyAction::Quit));
    }

    #[test]
    fn ignores_unknown_bytes_and_recovers_after_bad_escape() {
        let mut parser = EscapeParser::default();
        assert_eq!(parser.push(b'x'), None);
        assert_eq!(parser.push(0x1b), None);
        assert_eq!(parser.push(b'x'), None);
        assert_eq!(parser.push(b' '), Some(KeyAction::Stop));
        assert_eq!(parser.push(0x1b), None);
        assert_eq!(parser.push(b'['), None);
        assert_eq!(parser.push(b'Z'), None);
        assert_eq!(parser.push(b'q'), Some(KeyAction::Quit));
    }
}
