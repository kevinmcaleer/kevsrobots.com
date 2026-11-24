This folder is intentionally empty of data files.

The DuckDB course makes its own sample data. In lesson 1 (`01_installing.md`) learners save a short
script, `make_data.py`, which writes three CSV files into a `data/` folder in their own project:

- `readings.csv`: a sensor log from three robots (3,456 rows)
- `parts.csv`: a small parts inventory
- `print_jobs.csv`: a 3D print job history

The script uses a fixed random seed, so every learner gets identical files and the numbers printed in
the lessons match. Nothing needs to be downloaded, and nothing in this folder is used by the lessons.
