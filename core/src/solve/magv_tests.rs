//! P0-MAGV V3: the step counters over a directory of random coil boards (the compiled programs written by
//! test/core/magv/run-board.ts --dump). Ignored: it needs MAGV_BOARDS=<dir>.

use crate::engine::Engine;
use crate::program::Program;

#[test]
#[ignore = "needs MAGV_BOARDS=<dir of .f64 and .sched files> (test/core/magv/run-board.ts --dump)"]
fn magv_no_extrapolated_step_in_a_partition_holding_a_coil_over_random_boards() {
    let dir = std::env::var("MAGV_BOARDS").expect("MAGV_BOARDS");
    let mut names: Vec<_> = std::fs::read_dir(&dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "f64")).collect();
    names.sort();
    let (mut boards, mut bad, mut steps_total) = (0usize, Vec::new(), 0.0f64);
    for path in names {
        let bytes = std::fs::read(&path).unwrap();
        let data: Vec<f64> = bytes.chunks_exact(8).map(|c| f64::from_le_bytes(c.try_into().unwrap())).collect();
        let sched = std::fs::read_to_string(path.with_extension("sched")).unwrap();
        let mut lines = sched.lines();
        let src: u32 = lines.next().unwrap().parse().unwrap();
        let off_at: f64 = lines.next().unwrap().parse().unwrap();
        let stops: Vec<f64> = lines.next().unwrap().split_whitespace().map(|x| x.parse().unwrap()).collect();
        let program = Program::parse(&data).unwrap();
        let mut e = Engine::load(&program).unwrap();
        crate::solve::stepped::EXTRAPOLATED.with(|c| c.set(0));
        crate::solve::stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
        let mut off = false;
        for t in stops {
            e.advance(t).unwrap();
            if !off && t >= off_at {
                e.set_params(&[(src, 0, 0.0)]).unwrap();
                off = true;
            }
        }
        boards += 1;
        steps_total += e.stat(10);
        let (a, b) = (crate::solve::stepped::EXTRAPOLATED.with(|c| c.get()), crate::solve::stepped::EXTRAP_TRBDF2.with(|c| c.get()));
        if a + b != 0 {
            bad.push(format!("{} trap {a} trbdf2 {b}", path.display()));
        }
    }
    eprintln!("MAGV counters: {boards} boards, {steps_total} steps, {} boards with an extrapolated step", bad.len());
    assert!(bad.is_empty(), "{bad:?}");
}
