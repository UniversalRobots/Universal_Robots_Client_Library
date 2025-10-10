import pathlib

URCL_PATH = pathlib.Path(__file__).parent.parent.parent.resolve()

PKG_PATH = [i for i in pathlib.Path(URCL_PATH.as_posix()).glob("**/data_package.cpp")]

assert len(PKG_PATH) == 1

PKG_PATH = PKG_PATH[0].resolve()

OUTPUT_PATH = pathlib.Path(__file__).parent.resolve().as_posix() + "/exhaustive_rtde_output_recipe.txt"

with open(PKG_PATH) as pkg_file:
    save_outputs = False
    outputs = []
    while True:
        line = pkg_file.readline()
        if not line:
            break
        if "// INPUT / OUTPUT" in line:
            save_outputs = True
        if save_outputs:
            if "//" not in line and len(line) > 1:
                outputs.append(line.split('"')[1] + "\n")
        if "// NOT IN OFFICIAL DOCS" in line:
            break

    with open(OUTPUT_PATH, "w") as output_file:
        output_file.writelines(outputs)
