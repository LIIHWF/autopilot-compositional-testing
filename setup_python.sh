cd patch/PythonAPI-2021.3/
python3 -m pip install -r requirements.txt --user .
cd -
python3 -m pip install -r requirements.txt
bazel build //...