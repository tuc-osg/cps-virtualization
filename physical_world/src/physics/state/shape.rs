use dimensioned::si;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Sphere {
    pub radius: si::Meter<f64>,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Shape {
    Sphere(Sphere),
    None,
}

impl Default for Shape {
    fn default() -> Self {
        Shape::None
    }
}

impl std::ops::Add<Shape> for Shape {
    type Output = Shape;
    fn add(self, rhs: Shape) -> Shape {
        match self {
            Shape::Sphere(me) => match rhs {
                Shape::Sphere(other) => Shape::Sphere(Sphere {
                    radius: me.radius + other.radius,
                }),
                Shape::None => self,
            },
            Shape::None => self,
        }
    }
}

impl std::ops::Sub<Shape> for Shape {
    type Output = Shape;
    fn sub(self, rhs: Shape) -> Shape {
        match self {
            Shape::Sphere(me) => match rhs {
                Shape::Sphere(other) => Shape::Sphere(Sphere {
                    radius: me.radius - other.radius,
                }),
                Shape::None => self,
            },
            Shape::None => rhs,
        }
    }
}

impl std::fmt::Display for Shape {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Shape::Sphere(_) => write!(f, "Sphere"),
            Shape::None => write!(f, "None"),
        }
    }
}
