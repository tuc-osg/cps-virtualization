use log::LevelFilter;
use log4rs::append::file::FileAppender;
use log4rs::config::{Appender, Config, Root};
use log4rs::encode::pattern::PatternEncoder;

pub fn init_log(filename: &str) {
    let logfile_res = FileAppender::builder()
        .append(false)
        .encoder(Box::new(PatternEncoder::new("{l} - {m}\n")))
        .build(filename);
    let logfile = match logfile_res {
        Ok(lf) => lf,
        Err(e) => panic!("Encountered error while creating {filename}:\n {e}"),
    };
    let config_res = Config::builder()
        .appender(Appender::builder().build("logfile", Box::new(logfile)))
        .build(
            Root::builder()
                .appender("logfile")
                .build(LevelFilter::Debug),
        );
    let config = match config_res {
        Ok(cfg) => cfg,
        Err(e) => panic!("Encountered error while creating config for {filename}:\n {e}"),
    };
    match log4rs::init_config(config) {
        Ok(_) => (),
        Err(e) => panic!("Encountered error while initializing config for {filename}:\n {e}"),
    };
}
