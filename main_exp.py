from Experiment import ExperimentMaster

CFG_FILE ="/home/zzefduran/code/newBenchTest/Control_Experiments/configs/config_ktau_8064.yaml"

print(f"this is the path: {CFG_FILE}")
exp = ExperimentMaster(cfg_path=CFG_FILE)

exp.run_exp()