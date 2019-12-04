import os
import glob
import shutil


def read_task_file(file_name, imp_lines):
    file = open(file_name)
    # imp_lines = []
    for line in file.readlines():
        strs = line.strip("\n").split(" ")
        if strs[0] == "import" or strs[0] == "from":
            tem = strs[1].split(".")
            if len(tem) > 1:
                imp_line = "."
                for s in tem:
                    imp_line += "/"+s
                imp_lines.append(imp_line+".py")
                read_task_file(imp_line+".py", imp_lines)


def save_files(path, files):
    for file in files:
        name_list = file.split("/")[1:]
        target_path = path+"/".join(name_list[:-1])
        if not os.path.exists(target_path):
            os.makedirs(target_path)
        shutil.copy(file, path+"/".join(name_list))


def main():
    files = []
    read_task_file("./examples/TicTacToe.py", files)
    files.sort()
    print(files)
    save_files("./data/test/", files)
