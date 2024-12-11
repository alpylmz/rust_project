mod input;
mod helper;
mod config;
mod ast;
mod examples;
use examples::ex1;
use examples::rnea;

use crate::ast::ASTNode;



fn main() {
    //ex1::ex1();

    let qsin = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("qsin");
    let qcos = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("qcos");
    let q = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("q");
    let v = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("v");
    let a = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("a");


    rnea::rnea(qsin, qcos, q, v, a);

}
