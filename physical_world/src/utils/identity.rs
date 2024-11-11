pub trait Identity {
    fn get_identity(&self) -> &'static str;
}
