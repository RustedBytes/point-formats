use std::io::Cursor;
fn main() {
    let mut data = Cursor::new(Vec::new());
    let _reader = e57::E57Reader::new(&mut data);
}
