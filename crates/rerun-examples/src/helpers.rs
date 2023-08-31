use baby_shark::exports::nalgebra;

pub fn parse_color(color_str: &str) -> [f32; 4] {
    let colors: Vec<&str> = color_str.split(",").collect();
    let colors: Vec<f32> = colors
        .iter()
        .map(|c| c.parse::<f32>().unwrap() / 255.0)
        .collect();

    [
        *colors.get(0).unwrap_or(&1.0),
        *colors.get(1).unwrap_or(&1.0),
        *colors.get(2).unwrap_or(&1.0),
        *colors.get(3).unwrap_or(&1.0),
    ]
}
pub fn parse_radii_errors(radii_error_str: &str) -> Vec<(f32, f32)> {
    let components: Vec<&str> = radii_error_str.split(",").collect();

    let maps = components
        .iter()
        .map(|c| {
            let c2: Vec<&str> = c.split(':').collect();
            (c2[0].parse().unwrap(), c2[1].parse().unwrap())
        })
        .collect();
    maps
}

pub fn parse_point(point_str: &str) -> nalgebra::Point3<f32> {
    let components: Vec<&str> = point_str.split(",").collect();

    assert_eq!(3, components.len());

    nalgebra::Point3::new(
        components[0].parse().unwrap(),
        components[1].parse().unwrap(),
        components[2].parse().unwrap(),
    )
}

pub fn parse_vector(point_str: &str) -> nalgebra::Vector3<f32> {
    let components: Vec<&str> = point_str.split(",").collect();

    assert_eq!(3, components.len());

    nalgebra::Vector3::new(
        components[0].parse().unwrap(),
        components[1].parse().unwrap(),
        components[2].parse().unwrap(),
    )
}
