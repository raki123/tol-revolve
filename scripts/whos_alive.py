print("exp,run,births,robot_id")


def process_offline_dir(exp):
    gradual = "gradual" in exp
    birth_fac = 18 if exp == "plus-gradual_more" else 15

    with open('/media/expdata/output/'+exp+'/generations.csv', 'rb') as f:
        f.readline()

        for line in f:
            run, gen, robot_id, _, _, _, _ = line.split(',')

            if gradual:
                births = int(gen) + birth_fac
            else:
                births = int(gen) * 15 + 15

            print("%s,%s,%d,%s" % (exp, run, births, robot_id))


def process_online_dir(d):
    with open('/media/expdata/online-output/'+d+'/fitness.csv') as f:
        f.readline()

        cur_births = 0
        cur_t = ''

        for line in f:
            run, t_sim, births, robot_id, _, _, _, _, _ = line.split(',')

            if t_sim != cur_t and births == cur_births:
                continue

            cur_t = t_sim
            cur_births = births

            print("embodied,%s,%s,%s" % (run, births, robot_id))

process_offline_dir('plus')
process_offline_dir('plus-gradual')
process_offline_dir('plus-gradual-more')
process_online_dir('d0-5')
process_online_dir('d6-21')
process_online_dir('d23-31')
