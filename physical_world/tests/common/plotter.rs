use plotters::prelude::*;

pub struct DataSeries {
    pub name: String,
    pub data: Vec<(f64, f64)>,
}

pub trait Plotter {
    fn plot(&self, filename: &str, title: &str);
}

impl Plotter for Vec<DataSeries> {
    fn plot(&self, filename: &str, title: &str) {
        let mut max_x: f64 = self[0].data[0].0;
        let mut min_x: f64 = self[0].data[0].0;
        let mut max_y: f64 = self[0].data[0].1;
        let mut min_y: f64 = self[0].data[0].1;
        for series in self {
            for (x, y) in &series.data {
                if *x > max_x {
                    max_x = *x;
                }
                if *x < min_x {
                    min_x = *x;
                }
                if *y > max_y {
                    max_y = *y;
                }
                if *y < min_y {
                    min_y = *y;
                }
            }
        }
        let colors: [RGBColor; 6] = [
            RGBColor(0, 0, 255),
            RGBColor(255, 0, 0),
            RGBColor(6, 85, 53),
            RGBColor(0, 0, 0),
            RGBColor(255, 0, 255),
            RGBColor(128, 0, 128),
        ];
        let legends = colors.map(|c| move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], c));
        let root = BitMapBackend::new(filename, (640, 480)).into_drawing_area();
        root.fill(&WHITE).unwrap();
        let mut chart = ChartBuilder::on(&root)
            .caption(title, ("sans-serif", 30).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(40)
            .build_cartesian_2d(min_x..max_x, min_y..max_y)
            .unwrap();
        chart
            .configure_mesh()
            .x_desc("Time")
            .y_desc("Location")
            .draw()
            .unwrap();
        for (i, series) in self.into_iter().enumerate() {
            chart
                .draw_series(LineSeries::new(series.data.clone(), &colors[i]))
                .unwrap()
                .label(series.name.clone())
                .legend(legends[i]);
        }
        chart
            .configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw()
            .unwrap();
        root.present().unwrap();
    }
}
