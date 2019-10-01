import json
import argparse
import os
import shutil
import random
import cv2

def touch(path):
    if os.path.isfile(path):
        pass
    else:
        with open(path, "w") as f:
            pass

class AnnotationInfo:
    def __init__(self,image_count):
        self.image_count = image_count
        self.rects = []
    def addRect(self,rect):
        self.rects.append(rect)
    def getRects(self):
        return self.rects
    def getImagePath(self):
        return ".raw/"+str(self.image_count)+".png"

class Rect:
    def __init__(self,image_count,rect_count,x,y,width,height):
        self.image_count = image_count
        self.rect_count = rect_count
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    def getImageFilename(self):
        ret = str(self.image_count)+"_"+str(self.rect_count)+".png"
        return ret
    def getRawImageFilename(self):
        ret = str(self.image_count)+".png"
        return ret

class DatasetGenerator:
    def __init__(self):
        f = open('annotation.json', 'r')
        self.annotation = json.load(f)
        self.annotation_data = []
        self.class_and_id = []
        for image_count in self.annotation:
            info = AnnotationInfo(image_count)
            rects = self.annotation[image_count]
            for rect in rects:
                for rect_count in rect:
                    x = rect[rect_count]["rect"]["x"]
                    y = rect[rect_count]["rect"]["y"]
                    width = rect[rect_count]["rect"]["width"]
                    height = rect[rect_count]["rect"]["height"]
                    obj_rect = Rect(image_count,rect_count,x,y,width,height)
                    info.addRect(obj_rect)
            self.annotation_data.append(info)
    def prepare(self):
        ret = []
        files = os.listdir(".")
        files_dir = [f for f in files if os.path.isdir(os.path.join(".", f))]
        id = 0
        rects = []
        for splited_dir in files_dir:
            if splited_dir == ".raw":
                continue
            else:
                files_in_splited_dir = os.listdir(splited_dir)
                self.class_and_id.append((splited_dir,id))
                image_files = [f for f in files_in_splited_dir if os.path.isfile(os.path.join(splited_dir, f))]
                for image in image_files:
                    query_result = self.queryRectFromSplitedImageFilename(image)
                    if query_result[0] == True:
                        rects.append((id,query_result[1]))      
                id = id + 1
        return rects
    def queryRectFromSplitedImageFilename(self,splited_image_filename):
        for data in self.annotation_data:
            rects = data.getRects()
            for rect in rects:
                if splited_image_filename == rect.getImageFilename():
                    return (True,rect)
        return (False,Rect())

class DarknetDatasetGenerator(DatasetGenerator):
    def generate(self,test_ratio):
        self.test_ratio = test_ratio
        if os.path.exists(os.path.abspath("../dataset_darknet")):
            shutil.rmtree(os.path.abspath("../dataset_darknet"))
        os.mkdir("../dataset_darknet")
        os.mkdir("../dataset_darknet/dataset")
        os.mkdir("../dataset_darknet/backup")
        self.rects = self.prepare()
        self.generateNameFile()
        self.generateDotData()
        self.copyData()
        self.splitDataset()
    def splitDataset(self):
        touch("../dataset_darknet/train.txt")
        touch("../dataset_darknet/test.txt")
        raw_images = []
        for rect in self.rects:
            raw_images.append(rect[1].getRawImageFilename())
        raw_images = set(raw_images)
        prefix = os.path.abspath("../dataset_darknet/dataset")
        print(prefix)
        for raw_image in raw_images:
            if random.uniform(0,1) < self.test_ratio:
                with open("../dataset_darknet/test.txt", 'a') as f:
                    f.write(prefix+"/"+raw_image+"\n")
            else:
                with open("../dataset_darknet/train.txt", 'a') as f:
                    f.write(prefix+"/"+raw_image+"\n")  

        #random.uniform
    def generateNameFile(self):
        touch('../dataset_darknet/dataset.names')
        f = open('../dataset_darknet/dataset.names','w')
        for data in self.class_and_id:
            f.write(data[0]+"\n")
        f.close()
    def generateDotData(self):
        touch('../dataset_darknet/dataset.data')
        f = open('../dataset_darknet/dataset.data','w')
        f.write("classes= "+str(len(self.class_and_id))+"\n")
        f.write("train  = " + os.path.abspath("../dataset_darknet/train.txt") + "\n")
        f.write("valid  = " + os.path.abspath("../dataset_darknet/test.txt") + "\n")
        f.write("names = " + os.path.abspath("../dataset_darknet/dataset.names") + "\n")
        f.write("backup = " + os.path.abspath("../dataset_darknet/backup") + "\n")
        f.close()
    def copyData(self):
        raw_images = []
        for rect in self.rects:
            raw_images.append(rect[1].getRawImageFilename())
        raw_images = set(raw_images)
        for raw_image in raw_images:
            shutil.copyfile(".raw/"+raw_image, "../dataset_darknet/dataset/"+raw_image)
            name, ext = os.path.splitext("../dataset_darknet/dataset/"+raw_image)
            touch(name+".txt")
        for rect in self.rects:
            raw_image = rect[1].getRawImageFilename()
            img_data = cv2.imread("../dataset_darknet/dataset/"+raw_image, cv2.IMREAD_UNCHANGED)
            height, width, channels = img_data.shape[:3]
            name, ext = os.path.splitext("../dataset_darknet/dataset/"+raw_image)
            with open(name+".txt", 'a') as f:
                x = int(float(rect[1].x)+float(rect[1].width)/2.0)
                y = int(float(rect[1].y)+float(rect[1].height)/2.0)
                line = str(rect[0]) + " " + str(float(x)/width) + " " + str(float(y)/height) + " " + str(float(rect[1].width)/width) + " " + str(float(rect[1].height)/height) + "\n"
                f.write(line)

class KerasImageDataGeneratorDatasetGenerator(DatasetGenerator):
    def generate(self,test_ratio):
        self.test_ratio = test_ratio
        if os.path.exists(os.path.abspath("../Images")):
            shutil.rmtree(os.path.abspath("../Images"))
        os.mkdir("../Images")
        dirs = os.listdir(".")
        target_dirs = []
        for directory in dirs:
            if directory == "generate_dataset.py":
                continue
            if directory == "annotation.json":
                continue
            if directory == ".raw":
                continue
            target_dirs.append(directory)
        os.mkdir("../Images/Test")
        os.mkdir("../Images/Train")
        for target_dir in target_dirs:
            os.mkdir("../Images/Test/"+target_dir)
            os.mkdir("../Images/Train/"+target_dir)
            image_lists = os.listdir("./"+target_dir)
            for image in image_lists:
                if random.uniform(0,1) < self.test_ratio:
                    shutil.copy("./"+target_dir+"/"+image, "../Images/Test/"+target_dir+"/"+image)
                else:
                    shutil.copy("./"+target_dir+"/"+image, "../Images/Train/"+target_dir+"/"+image)

if __name__ == "__main__":
    formats = ["darknet","keras_image_data_generator"]
    parser = argparse.ArgumentParser(description='Dataset generator for Maritime RobotX Challenge.')
    parser.add_argument('format', help='Dataset format, now we can use darknet format,keras_image_data_generator format')
    parser.add_argument('test_ratio', help='Ratio of the test image')
    args = parser.parse_args()
    if args.format not in formats:
        print("invalid format")
    if args.format == "darknet":
        gen = DarknetDatasetGenerator()
        gen.generate(float(args.test_ratio))
    if args.format == "keras_image_data_generator":
        gen = KerasImageDataGeneratorDatasetGenerator()
        gen.generate(float(args.test_ratio))