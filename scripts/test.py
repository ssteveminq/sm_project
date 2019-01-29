import pynusmv

pynusmv.init.init_nusmv()
pynusmv.glob.load_from_file("robot_delivery.smv")
pynusmv.glob.compute_model()
Is_init=pynusmv.init.is_nusmv_init()

# print(Is_init)

fsm = pynusmv.glob.prop_database().master.bddFsm
print(fsm)
propDb=pynusmv.glob.prop_database()
print(propDb)
for prop in propDb:
    print(prop.type)
    if prop.type==pynusmv.prop.propTypes['LTL']:
        print("trueeee!!")
        spec =prop.exprcore
        print('Specification', str(spec))
        violating = pynusmv.mc.check_explain_ltl_spec(spec)
        print(violating)
        
# for prop n propDb:
    # if prop.type==propTypes[]
# prop = pynusmv.glob.prop_database()
# spec = prop.expr
# bdd = pynusmv.mc.eval_ctl_spec(fsm, spec) 
# print(bdd)


# pynusmv.mc.check_ltl_spec("! ((G F work_load >= 6) & (G work_load <= 20) & (G work_load > 0) )")
