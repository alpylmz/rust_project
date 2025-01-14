mod input;
mod helper;
mod config;
mod ast;
mod examples;
use examples::{ex1, inversempc, rnea, rneaderivatives};

use crate::ast::ASTNode;



fn main() {
    //ex1::ex1();

    let q = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("q");
    let qsin = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("qsin");
    let qs_sin = Matrix!(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ).define("qs_sin");
    let qcos = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("qcos");
    let qs_cos = Matrix!(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ).define("qs_cos");
    let v = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("v");
    let a = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("a");
    let us = Matrix!(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ).define("us");
    let xgoal = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("xgoal");


    //rnea::rnea(qsin, qcos, v, a);
    //inversempc::solver(q, qsin, qcos, us, xgoal);
    rneaderivatives::rneaderivatives(qsin, qcos, v, a);
}
