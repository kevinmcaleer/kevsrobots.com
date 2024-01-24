!#/bin/bash

# remove old index
echo "Removing old index"
rm -rf search.db

# create new index
echo "Creating new index"
python3 search/database.py
time python3 index.py

# run the app
# echo "Running the app"
# uvicorn app:app --reload

