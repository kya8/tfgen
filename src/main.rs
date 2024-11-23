use std::{io::stdin, process::ExitCode, str::FromStr};
use tfgen::{
    se3::{self, To7, SE3},
    TfGraph,
};
use nalgebra as na;
use owo_colors::OwoColorize;

fn main() -> ExitCode {
    let mut g = TfGraph::default();
    println!("{}", "Enter a command. h for help.".blue());

    for line in stdin().lines() {
        let line = line.expect("Error reading input");
        // allow blank lines
        if line.is_empty() {
            continue;
        }
        let Some(input) = parse_input(&line) else {
            eprintln!("{}", "Invalid input!".bright_red());
            continue;
        };

        match input {
            Input::Quit => break,
            Input::Reset => {
                g.reset();
                println!("{}", "Graph was reset.".blue());
            }
            Input::Help => print_help(),
            Input::Add { from, to, tf } =>
                if g.add_tf(from, to, tf).is_none() {
                    eprintln!("{}", "Could not add cyclic transform".bright_red());
                }
            Input::Query { from, to } => {
                if let Some((tf, path)) = g.query_tf(&from, &to) {
                    let mat: na::Matrix4<f64> = na::convert(tf);
                    println!("Transform from {} to {}: (Path: {})", from.green(), to.green(), path.join(" -> "));
                    println!("{mat}[x,y,z, qx,qy,qz,qw]: {:?}", tf.to7());
                } else {
                    eprintln!("No transform between {} and {}!", from.green(), to.green());
                }
            }
        }
    }

    ExitCode::SUCCESS
}

fn parse_csv<T: FromStr>(s: &str, delim: char) -> Result<Vec<T>, <T as FromStr>::Err> {
    s.trim_matches(['[', ']', ' '])
        .split(delim)
        .map(|x| x.trim().parse())
        .collect()
}

#[derive(PartialEq, Debug)]
enum Input {
    Add { from: String, to: String, tf: SE3 },
    Query { from: String, to: String }, // Could use &str here.
    Reset,
    Quit,
    Help,
    //Show
    //Load file
    //Export
}

fn parse_input(line: &str) -> Option<Input> {
    match line.trim() {
        "q" => Some(Input::Quit),
        "r" => Some(Input::Reset),
        "h" | "help" => Some(Input::Help),
        s => {
            let (src, rem) = s.split_once("->")?;
            if let Some((dst, tf)) = rem.split_once(':') {
                Some(Input::Add {
                    from: src.trim().to_owned(),
                    to: dst.trim().to_owned(),
                    tf: se3::from_array(parse_csv(tf, ',').ok()?.as_slice())?,
                })
            } else {
                Some(Input::Query {
                    from: src.trim().to_owned(),
                    to: rem.trim().to_owned(),
                })
            }
        }
    }
}

fn print_help() {
    println!("{} Source -> Target: tx, ty, tz, qx, qy, qz, qw | tx, ty, tz | qx, qy, qz, qw | 3x3 mat | 4x4 mat", "* Add a transform:".blue().bold());
    println!("{} Source -> Target", "* Query transform:".blue().bold());
    println!("{} r", "* Remove all transforms:".blue().bold());
    println!("{} q", "* Quit:".blue().bold());
    println!("{} h | help", "* Help:".blue().bold());
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_input() {
        let inputs = [
            ("q", Input::Quit),
            ("r ", Input::Reset),
            ("help", Input::Help),
            (
                "Alice -> Bob : 0,0,0",
                Input::Add {
                    from: "Alice".to_owned(),
                    to: "Bob".to_owned(),
                    tf: SE3::identity(),
                },
            ),
            (
                "Bob -> Alice",
                Input::Query {
                    from: "Bob".to_owned(),
                    to: "Alice".to_owned(),
                },
            ),
        ];

        let bad_inputs = [
            "",
            "qr",
            //"q ",
            "Alice -> Bob : 0,0,0,0,0",
        ];

        for (line, result) in inputs {
            assert_eq!(parse_input(line), Some(result));
        }
        for line in bad_inputs {
            assert_eq!(parse_input(line), None);
        }
    }
}
