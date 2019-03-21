# coding: utf-8

import os

channel_map2 = {
	'px' : 'Xposition',
	'py' : 'Yposition',
	'pz' : 'Zposition',
	'rx' : 'Xrotation',
	'ry' : 'Yrotation',
	'rz' : 'Zrotation',
}

def jointWrite(file, joint):
	if joint.name:
		file.write("{\n")
		offsetString = "OFFSET "+str(joint.offset[0])+" "+str(joint.offset[1])+" "+str(joint.offset[2]) + "\n"
		channelString = "CHANNELS " + str(len(joint.channels))
		for i in xrange(len(joint.channels)):
			channelString += " " + channel_map2[str(joint.channels[i])]
		channelString += "\n"
		file.write(offsetString)
		file.write(channelString)
		if joint.children:
			for j in xrange(len(joint.children)):
				jointName = "JOINT " + joint.children[j].name + "\n"
				file.write(jointName)
				jointWrite(file, joint.children[j])
		else:
			file.write("End Site\n")
			file.write("{\n")
			tailString = "OFFSET "+str(joint.tail[0])+" "+str(joint.tail[1])+" "+str(joint.tail[2]) + "\n"
			file.write(tailString)
			file.write("}\n")	
		file.write("}\n")


def bvhWrite( retargetFile, loader , N ):
    file = open(retargetFile, 'w+')
    file.write("HIERARCHY\n")
    joint = loader.root
    if joint.name:
        root = "ROOT " + joint.name + "\n"
        file.write(root)
        jointWrite(file, joint)
        file.write("MOTION\n")
        frameNum = "Frames: " + str(N)+"\n"
        file.write(frameNum)
        frameTime = "Frame Time: " + str(loader.frame_interval) +"\n"
        file.write(frameTime)
        for frame in xrange( N ):
            motionString = ""
            for channelNum in xrange(len(loader.channels)):
                motionString += str(round(loader.channels[channelNum].motion[frame],3))+" "
            motionString += "\n"
            file.write(motionString)
        file.close()





######################################
if __name__ == '__main__':
    main()