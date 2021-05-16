## Precision-Recall curve
Save the data in a text file, the first column is the predicted score, and the second column is the ground truth.
```bash
python3 pr_curve.py
```

## Top-k Recall curve
Save the data in a text file, the first column is the target index, and the remaining columns are candidate indexes.
```bash
python3 recall.py
```

## Generate sample list
```bash
python3 gen_pairs.py
```