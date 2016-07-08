import shutil
import os

with open('list.txt', 'r') as f:
    for line in f:
        data = line.strip()
        exp, robot_id = data.split()
        robot_filename = "robot_"+robot_id+".sdf"

        if exp == 'embodied':
            path = "/media/expdata/online-output"
            if os.path.isfile(path+"/d0-5/"+robot_filename):
                path += "/d0-5"
            elif os.path.isfile(path+"/d6-21/"+robot_filename):
                path += "/d6-21"
            else:
                path += "/d23-31"
        else:
            path = "/media/expdata/output/"+exp

        out_name = '/home/elte/mt/Thesis/robot_images/morph/'+exp+'_'+robot_id+'.png'
        if os.path.exists(out_name):
            continue

        _ = raw_input("Enter something when ready..")

        shutil.copy(path + "/" + robot_filename, "/home/elte/.gazebo/models/test_bot/model.sdf")
        with open('/home/elte/mt/tol/tools/screenshot-tools/fname', 'w') as fw:
            fw.write('/home/elte/mt/Thesis/robot_images/morph/'+exp+'_'+robot_id+'.png')

        print("Robot copied.")
