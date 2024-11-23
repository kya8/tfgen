use std::{io::stdin, process::ExitCode, str::FromStr};
use tfgen::{
    se3::{self, To7, SE3},
    TfGraph,
};
use nalgebra as na;

fn main() -> ExitCode {
    let mut g = TfGraph::default();
    println!("Enter a command. h for help.");

    for line in stdin().lines() {
        let line = line.expect("Error reading input");
        let Some(input) = parse_input(&line) else {
            eprintln!("Invalid input!");
            continue;
        };

        match input {
            Input::Quit => break,
            Input::Reset => {
                g.reset();
                println!("Graph was reset.");
            }
            Input::Help => print!("{HELP_TEXT}"),
            Input::Add { from, to, tf } =>
                if g.add_tf(from, to, tf).is_none() {
                    eprintln!("Could not add cyclic transform");
                }
            Input::Query { from, to } => {
                if let Some(tf) = g.query_tf(&from, &to) {
                    let mat: na::Matrix4<f64> = na::convert(tf);
                    println!("Transform from \"{from}\" to \"{to}\":");
                    println!("{mat}[x,y,z, qx,qy,qz,qw]: {:?}", tf.to7());
                } else {
                    eprintln!("No transform between \"{from}\" and \"{to}\"!");
                }
            }
        }
    }

    ExitCode::SUCCESS
}

fn parse_csv<T: FromStr>(s: &str, delim: char) -> Result<Vec<T>, <T as FromStr>::Err> {
    s.trim_matches(['[', ']'])
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

const HELP_TEXT: &'static str =
"\
Add a transform: Source -> Target: tx, ty, tz, qx, qy, qz, qw | tx, ty, tz | qx, qy, qz, qw | 3x3 mat | 4x4 mat
Query transform: Source -> Target
Remove all transforms: r
Quit: q
Help: h | help
";

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
