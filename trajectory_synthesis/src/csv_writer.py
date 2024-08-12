import os
import pandas as pd


class CsvWriter:
    def __init__(self, dir) -> None:
        self.dir = dir
        self.data: dict = None

    def set_data(self, data):
        self.data = data

    def update_data(self, data):
        if self.data is None:
            self.data = data
        else:
            self.data.update(data)

    def write_data_old_format(self, name):
        old_data = {key[:-4]: val for key, val in self.data.items() if key.endswith("_pos")}
        dirname = os.path.join(self.dir, "old_format")
        os.makedirs(dirname, exist_ok=True)
        self._save(old_data, dirname, name, True)

    def _save(self, data: dict, dirname: str, name: str, index: bool):
        pd_dict = pd.DataFrame.from_dict(data)
        path = os.path.join(dirname, f"{name}.csv")
        pd_dict.to_csv(path, index=index)
        print(f"Saved file {path}")
        return path

    def write_data(self, name):
        if self.data is None:
            raise ValueError("self.data is None, use method self.set_data to set it first")
        path = self._save(self.data, self.dir, name, False)
        return path
        # TODO: remove this once new cliapp is stable
        # self.write_data_old_format(name)
