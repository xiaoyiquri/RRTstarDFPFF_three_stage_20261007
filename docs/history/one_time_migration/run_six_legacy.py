#!/usr/bin/env python3
"""Run six YAML environments with authoritative float8 discrete physical validation."""
import argparse
import csv
import datetime
import hashlib
import json
import pathlib
import platform
import re
import subprocess
import sys

CASES = {
    "uni_bugtrap": ("unicycle1_v0/bugtrap_0.yaml", "unicycle1_v0"),
    "uni_wall": ("unicycle1_v2/wall_0.yaml", "unicycle1_v2"),
    "quad2d_bugtrap": ("quad2d_v0/quad_bugtrap.yaml", "quad2d_v0"),
    "quad2d_hole": ("quad2d_v0/fall_through.yaml", "quad2d_v0"),
    "quad3d_block": ("quadrotor_v0/quad_one_obs.yaml", "quad3d_v0"),
    "quad3d_window": ("quadrotor_v0/window.yaml", "quad3d_v0"),
}

# These two profiles intentionally override DynoBench's published model values
# for the paper experiment. Quad2D is identical to the upstream model and is
# resolved directly from <dynobench-source>/models.
PAPER_MODELS = {"unicycle1_v0", "unicycle1_v2", "quad3d_v0"}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace(text, key, value):
    result, count = re.subn(r"(?m)^  " + re.escape(key) + r":.*$",
                            lambda _: "  " + key + ": " + value, text)
    if count != 1:
        raise ValueError("Expected exactly one configuration key: " + key)
    return result


def execute(command, log, timeout):
    with log.open("w", encoding="utf-8") as stream:
        stream.write("COMMAND: " + json.dumps([str(x) for x in command]) + "\n")
        stream.flush()
        try:
            result = subprocess.run([str(x) for x in command], stdout=stream,
                                    stderr=subprocess.STDOUT, timeout=timeout, check=False)
            return result.returncode
        except subprocess.TimeoutExpired:
            stream.write("\nTIMEOUT: process terminated\n")
            return 124


def main():
    project = pathlib.Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dynobench-source", type=pathlib.Path, required=True,
                        help="Existing YAML data source only; no DynoBench binaries are invoked.")
    parser.add_argument("--build-dir", type=pathlib.Path, default=project / "build_simd")
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--first-solution", action="store_true")
    parser.add_argument("--df-mode", choices=("paper_r2", "full_df"), default="paper_r2")
    parser.add_argument("--cases", nargs="+", choices=list(CASES), default=list(CASES))
    args = parser.parse_args()
    if args.runs < 1 or not 0 < args.timeout < float("inf") or not 0 <= args.seed <= 4294967295 - args.runs + 1:
        parser.error("Invalid runs, timeout or seed")
    source, build = args.dynobench_source.resolve(), args.build_dir.resolve()
    binary = build / ("benchmark.exe" if sys.platform == "win32" else "benchmark")
    if not binary.is_file(): parser.error("Missing executable: " + str(binary))
    selected = list(dict.fromkeys(args.cases))
    for case in selected:
        env, model = CASES[case]
        model_file = (project / "benchmark/models/paper" / (model + ".yaml")).resolve()
        model_root = project / "benchmark/models/paper" if model in PAPER_MODELS else source / "models"
        for file in (source / "envs" / env, model_root / (model + ".yaml")):
            if not file.is_file(): parser.error("Missing YAML: " + str(file))
    output = (args.output or project / "results" / datetime.datetime.now().strftime("simd_%Y%m%d_%H%M%S_%f")).resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(validation_backend="simd_discrete_float8", geometry="sphere_union",
                    profile="paper_fair", df_mode=args.df_mode,
                    boundary_contract="reduced_flat_A_prime" if args.df_mode == "paper_r2" else "full_physical",
                    solver_precision="float64", validation_precision="float32", time_root_precision="long_double",
                    external_checks=0, dynobench_verified=False, continuous_collision_certified=False,
                    runs=args.runs, timeout=args.timeout, seed=args.seed, cases=selected,
                    first_solution=args.first_solution, platform=platform.platform(),
                    binary_sha256=digest(binary), inputs={})
    summaries=[]
    print("Float8 discrete physical validation; no scalar/FCL/check_trajectory calls", flush=True)
    for case in selected:
        env, model = CASES[case]
        config_root = project / "benchmark/config"
        if args.df_mode == "full_df":
            config_root = config_root / "full_df"
        original=config_root / (case+".yaml")
        model_file = (project / "benchmark/models/paper" / (model + ".yaml")).resolve()
        text=original.read_text(encoding="utf-8")
        replacements = {"source":json.dumps(source.as_posix()),
                          "output_directory":json.dumps((output/case).as_posix()),
                          "runs":str(args.runs),"timeout":str(args.timeout),"seed":str(args.seed),
                          "stop_at_first_solution":str(args.first_solution).lower()}
        if model in PAPER_MODELS:
            replacements["model_file"] = json.dumps(model_file.as_posix())
        for key, value in replacements.items():
            text=replace(text,key,value)
        config=output/(case+".yaml");config.write_text(text,encoding="utf-8")
        input_model = (project / "benchmark/models/paper" / (model + ".yaml")) \
            if model in PAPER_MODELS else (source / "models" / (model + ".yaml"))
        manifest["inputs"][case]={str(f):digest(f) for f in (original,source/"envs"/env,input_model,config)}
        print(case+": starting",flush=True)
        code=execute([binary,"--config",config],output/(case+".log"),args.runs*(args.timeout+15)+30)
        records=[]
        results=output/case/"results.csv"
        if results.is_file():
            with results.open(newline="",encoding="utf-8") as stream: records=list(csv.DictReader(stream))
        successes=sum(r.get("success")=="1" and r.get("validation_backend")=="simd_discrete_float8" for r in records)
        summaries.append(dict(environment=case,runs=args.runs,sampled_successes=successes,
                              external_checks=0,return_code=code,validation_backend="simd_discrete_float8"))
        print(f"{case}: sampled success {successes}/{args.runs}, exit={code}",flush=True)
    (output/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    with (output/"summary.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(summaries[0]));writer.writeheader();writer.writerows(summaries)
    print("Results: "+str(output),flush=True)
    return 0 if all(s["return_code"]==0 and s["sampled_successes"]==s["runs"] for s in summaries) else 2


if __name__ == "__main__":
    sys.exit(main())
