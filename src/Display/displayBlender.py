'''
'''
import bpy
import os
import platform

def splitFilePath(path):
    if "Windows" in platform.system() or "CYGWIN" in platform.system():
        split = path.split("\\")
    else:
        split = path.split("/")

    return split

# Load objects from scene file
f = open(os.environ["SCENE_PATH"], 'r')
savePath = os.environ["SAVE"]
for line in f:
    line = line.strip('\n')
    spSplit = splitFilePath(os.environ["SCENE_PATH"]);

    spSplit[-1]="objs/"+line
    line = "/".join(spSplit)
    bpy.ops.import_scene.obj(filepath=line)
f.close()

# Center screen to all objects
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        for region in area.regions:
            if region.type == 'WINDOW':
                override = {'area': area, 'region': region, 'edit_object': bpy.context.edit_object}
                bpy.ops.view3d.view_all(override)

savePath = "/".join(splitFilePath(os.environ["SAVE"]))
bpy.ops.wm.save_as_mainfile(filepath=savePath)

