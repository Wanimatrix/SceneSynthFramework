'''
'''
import bpy
import os
import platform
import time

def splitFilePath(path):
    if "Windows" in platform.system() or "CYGWIN" in platform.system():
        split = path.split("\\")
    else:
        split = path.split("/")

    return split

# Load objects from scene file
f = open(os.environ["SCENE_PATH"]+"/scene.txt", 'r')
savePath = os.environ["SAVE"]
for line in f:
    line = line.strip('\n')
    # spSplit = splitFilePath(os.environ["SCENE_PATH"]);

    # spSplit[-1]="objs/"+line
    # line = "/".join(spSplit)
    bpy.ops.import_scene.obj(filepath=os.environ["SCENE_PATH"]+"/objs/"+line)
f.close()


ibsMaterial = bpy.data.materials['IBSColor']
objectMaterial = bpy.data.materials['SceneColor']

for obj in bpy.data.objects:
  if obj.type == 'MESH':
    if "ibs" in obj.name:
      obj.data.materials.append(ibsMaterial)
    else:
      obj.data.materials.append(objectMaterial)

# Center screen to all objects
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        for region in area.regions:
            if region.type == 'WINDOW':
                override = {'area': area, 'region': region, 'edit_object': bpy.context.edit_object}
                bpy.ops.view3d.view_all(override)


# Load objects from scene file
# f = open(os.environ["SCENE_PATH"]+"/samples/", 'r')

objSamplesColor = bpy.data.materials['ObjectSampleColor']
ibsSamplesColor = bpy.data.materials['IBSSampleColor']
bpy.ops.object.select_all(action='DESELECT')
# bpy.ops.mesh.primitive_cube_add(radius=0.05)#primitive_uv_sphere_add(size=0.05,segments=6,ring_count=4)
# sphere = bpy.context.object
# sphere.data.materials.append(objSamplesColor)

for fn in os.listdir(os.environ["SCENE_PATH"]+"/samples/"):
    print("Generating sampleset from file "+fn)
    f = open(os.environ["SCENE_PATH"]+"/samples/"+fn, 'r')
    sampleIdx = 0
    # bpy.ops.mesh.primitive_cube_add(radius=0.05)
    bpy.ops.mesh.primitive_uv_sphere_add(size=0.01,segments=6,ring_count=4)
    sphere = bpy.context.object
    sphere.data.materials.append(objSamplesColor)
    bpy.ops.object.select_all(action='DESELECT')
    # grp = bpy.data.groups.new(fn)
    # bpy.ops.object.add(type='EMPTY')
    # bpy.context.object.name = fn
    # prnt = bpy.context.object
    theObj = 0
    for line in f:
        # print("Sample "+str(sampleIdx))
        pos = line.split(",")
        # now = time.time()
        ob = sphere.copy()
        # print("Copying takes "+str(time.time()-now)+" sec")
        ob.location.x = float(pos[0])
        ob.location.y = -float(pos[2])
        ob.location.z = float(pos[1])
        # now = time.time()
        # ob.data = sphere.data.copy()
        # print("Data copying takes "+str(time.time()-now)+" sec")
        bpy.context.scene.objects.link(ob)
        # bpy.ops.mesh.primitive_ico_sphere_add(size=0.1,location=(float(pos[0]),float(pos[1]),float(pos[2])))
        objName = fn[0:-5]+"_"+str(sampleIdx)
        ob.name = objName
        # grp.objects.link(ob)
        # ob.hide = True
        # ob.parent = prnt
        ob.select = True
        bpy.context.scene.objects.active = ob
        # if "ibs" in objName:
        #     ob.data.materials.append(ibsSamplesColor)
        # else:
        #     ob.data.materials.append(objSamplesColor)
        sampleIdx+=1
    # bpy.context.scene.objects.active = prnt
    bpy.ops.object.join()
    bpy.context.object.name = fn
    bpy.ops.object.select_all(action='DESELECT')
    sphere.select = True
    bpy.ops.object.delete()
    f.close()
# bpy.context.scene.update()

savePath = "/".join(splitFilePath(os.environ["SAVE"]))
bpy.ops.wm.save_as_mainfile(filepath=savePath)

