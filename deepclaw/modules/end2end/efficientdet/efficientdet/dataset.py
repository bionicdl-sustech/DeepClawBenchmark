import os
import torch
import numpy as np

from torch.utils.data import Dataset, DataLoader
from pycocotools.coco import COCO
import cv2, json, copy

class WasteDataset(Dataset):
    """Waste dataset."""

    def __init__(self, root_dir, type="simple", set_name='train', transform=None):
        """
        Args:
            root_dir (string): COCO directory.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.root_dir = root_dir
        self.set_name = set_name
        self.transform = transform
        self.type = type

        if type == "simple" or type == "complex":
            self.data_dict = self.get_single_dicts()
        elif type == "all":
            self.data_dict = self.get_all_dicts()
        elif type == "recyclable":
            self.data_dict = self.get_recyclable_dicts()
        elif type == "complex_val":
            self.data_dict = self.get_val_dicts()

        self.load_classes()

    def load_classes(self):
        f = open(self.root_dir+"/category_names.txt",'r')
        for line in f.readlines():
            names = line.replace('\n','').replace('\'','').split(',')
            for i in range(1,len(names)):
                names[i] = names[i][1:]
        f.close()
        self.classes = {}
        for  i in range(len(names)):
            self.classes[names[i]]=i

        # also load the reverse (label -> name)
        self.labels = {}
        for key, value in self.classes.items():
            self.labels[value] = key

    def __len__(self):
        return len(self.data_dict)

    def __getitem__(self, idx):
        try:
            img = cv2.imread(self.data_dict[idx]["file_name"])
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = img.astype(np.float32) / 255.
        except:
            pass
        # if self.type == "complex_val":
            # annot = self.data_dict[idx]['annotations']
        # else:
        # annot = self.data_dict[idx]['annotations'].reshape([-1,5])

        annot = copy.deepcopy(self.data_dict[idx]['annotations'])
        annot = annot.reshape([-1,5])

        sample = {'img': img, 'annot': annot}
        if self.transform:
            sample = self.transform(sample)
        return sample

    def num_classes(self):
        return 204

    def get_single_dicts(self):
        dataset_dicts = []
        image_id_index = {}
        index = 0

        # initialte dataset dict with filename and image_id
        json_file = os.path.join(self.root_dir, self.type, self.set_name, 'train.json')
        with open(json_file) as f:
            file = json.load(f)
        images = file['images']
        for d in images:
            filename = os.path.join(self.root_dir, self.type, self.set_name, d['file_name'])
            if not os.path.exists(filename):
                print("Found not exiting file %s")
                continue
            record = {}
            record["file_name"] = filename
            record["image_id"] = d['image_id']
            record["height"] = d['height']
            record["width"] = d['width']
            record["annotations"] = np.zeros((0, 5))
            dataset_dicts.append(record)
            image_id_index.update({d['image_id']:index})
            index += 1
        # update annotations
        imgs_anns = file['annotations']
        for d in imgs_anns:
            image_id = d['image_id']
            if image_id in image_id_index:
                index = image_id_index[d['image_id']]
                annotation = np.zeros((1, 5))
                annotation[0, :4] = d['bbox']
                annotation[0, 4] = d['category_id']-1
                annotation[:, 2] = annotation[:, 0] + annotation[:, 2]
                annotation[:, 3] = annotation[:, 1] + annotation[:, 3]
                dataset_dicts[index]["annotations"] = np.append(dataset_dicts[index]["annotations"],annotation)
        return dataset_dicts

    def get_all_dicts(self):
        dataset_dicts = []
        image_id_index = {}
        index = 0

        # initialte dataset dict with filename and image_id
        json_file = os.path.join(self.root_dir, 'simple', self.set_name, 'train.json')
        with open(json_file) as f:
            file = json.load(f)
        images = file['images']
        for d in images:
            filename = os.path.join(self.root_dir, 'simple', self.set_name, d['file_name'])
            if not os.path.exists(filename):
                print("Found not exiting file %s")
                continue
            record = {}
            record["file_name"] = filename
            record["image_id"] = d['image_id']
            record["height"] = d['height']
            record["width"] = d['width']
            record["annotations"] = np.zeros((0, 5))
            dataset_dicts.append(record)
            image_id_index.update({d['image_id']:index})
            index += 1
        # update annotations
        imgs_anns = file['annotations']
        for d in imgs_anns:
            image_id = d['image_id']
            if image_id in image_id_index:
                idx = image_id_index[d['image_id']]
                annotation = np.zeros((1, 5))
                annotation[0, :4] = d['bbox']
                annotation[0, 4] = d['category_id']-1
                annotation[:, 2] = annotation[:, 0] + annotation[:, 2]
                annotation[:, 3] = annotation[:, 1] + annotation[:, 3]
                dataset_dicts[idx]["annotations"] = np.append(dataset_dicts[idx]["annotations"],annotation)
        
        # initialte dataset dict with filename and image_id
        json_file = os.path.join(self.root_dir, 'complex', self.set_name, 'train.json')
        with open(json_file) as f:
            file = json.load(f)
        images = file['images']
        for d in images:
            filename = os.path.join(self.root_dir, 'complex', self.set_name, d['file_name'])
            if not os.path.exists(filename):
                print("Found not exiting file %s")
                continue
            record = {}
            record["file_name"] = filename
            record["image_id"] = d['image_id']
            record["height"] = d['height']
            record["width"] = d['width']
            record["annotations"] = np.zeros((0, 5))
            dataset_dicts.append(record)
            image_id_index.update({d['image_id']:index})
            index += 1
        # update annotations
        imgs_anns = file['annotations']
        for d in imgs_anns:
            image_id = d['image_id']
            if image_id in image_id_index:
                index = image_id_index[d['image_id']]
                annotation = np.zeros((1, 5))
                annotation[0, :4] = d['bbox']
                annotation[0, 4] = d['category_id']-1
                annotation[:, 2] = annotation[:, 0] + annotation[:, 2]
                annotation[:, 3] = annotation[:, 1] + annotation[:, 3]
                dataset_dicts[index]["annotations"] = np.append(dataset_dicts[index]["annotations"],annotation)
        
        return dataset_dicts

    def get_recyclable_dicts(self):
        dataset_dicts = []
        image_id_index = {}
        index = 0

        # initialte dataset dict with filename and image_id
        json_file = os.path.join(self.root_dir, 'simple_recyclable', 'train.json')
        with open(json_file) as f:
            file = json.load(f)
        for d in file:
            filename = os.path.join(self.root_dir, 'simple_recyclable', d['file_name'][30:])
            if not os.path.exists(filename):
                print("Found not exiting file %s")
                continue
            record = {}
            record["file_name"] = filename
            record["image_id"] = d['image_id']
            record["height"] = d['height']
            record["width"] = d['width']
            record["annotations"] = np.zeros((0, 5))
    
            annotation = np.zeros((1, 5))
            annotation[0, :4] = d['annotations'][0]['bbox']
            annotation[0, 4] = d['annotations'][0]['category_id']
            annotation[:, 2] = annotation[:, 0] + annotation[:, 2]
            annotation[:, 3] = annotation[:, 1] + annotation[:, 3]
            record["annotations"] = np.append(record["annotations"],annotation)
            dataset_dicts.append(record)
        return dataset_dicts

    def get_val_dicts(self):
        json_file = self.label_path
        with open(json_file) as f:
            file = json.load(f)
        # initialte dataset dict with filename and image_id
        images = file['images']
        dataset_dicts = []
        image_id_index = {}
        index = 0
        for d in images:
            record = {}
            filename = os.path.join(self.root_dir, d['file_name'])
            record["file_name"] = filename
            record["image_id"] = d['image_id']
            record["height"] = d['height']
            record["width"] = d['width']
            record["annotations"] = np.zeros((1, 5))
            dataset_dicts.append(record)
            image_id_index.update({d['image_id']:index})
            index += 1
        return dataset_dicts

class CocoDataset(Dataset):
    def __init__(self, root_dir, set='train2017', transform=None):

        self.root_dir = root_dir
        self.set_name = set
        self.transform = transform

        self.coco = COCO(os.path.join(self.root_dir, 'annotations', 'instances_' + self.set_name + '.json'))
        self.image_ids = self.coco.getImgIds()

        self.load_classes()

    def load_classes(self):

        # load class names (name -> label)
        categories = self.coco.loadCats(self.coco.getCatIds())
        categories.sort(key=lambda x: x['id'])

        self.classes = {}
        self.coco_labels = {}
        self.coco_labels_inverse = {}
        for c in categories:
            self.coco_labels[len(self.classes)] = c['id']
            self.coco_labels_inverse[c['id']] = len(self.classes)
            self.classes[c['name']] = len(self.classes)

        # also load the reverse (label -> name)
        self.labels = {}
        for key, value in self.classes.items():
            self.labels[value] = key

    def __len__(self):
        return len(self.image_ids)

    def __getitem__(self, idx):

        img = self.load_image(idx)
        annot = self.load_annotations(idx)
        sample = {'img': img, 'annot': annot}
        if self.transform:
            sample = self.transform(sample)
        return sample

    def load_image(self, image_index):
        image_info = self.coco.loadImgs(self.image_ids[image_index])[0]
        path = os.path.join(self.root_dir, self.set_name, image_info['file_name'])
        img = cv2.imread(path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        return img.astype(np.float32) / 255.

    def load_annotations(self, image_index):
        # get ground truth annotations
        annotations_ids = self.coco.getAnnIds(imgIds=self.image_ids[image_index], iscrowd=False)
        annotations = np.zeros((0, 5))

        # some images appear to miss annotations
        if len(annotations_ids) == 0:
            return annotations

        # parse annotations
        coco_annotations = self.coco.loadAnns(annotations_ids)
        for idx, a in enumerate(coco_annotations):

            # some annotations have basically no width / height, skip them
            if a['bbox'][2] < 1 or a['bbox'][3] < 1:
                continue

            annotation = np.zeros((1, 5))
            annotation[0, :4] = a['bbox']
            annotation[0, 4] = self.coco_label_to_label(a['category_id'])
            annotations = np.append(annotations, annotation, axis=0)

        # transform from [x, y, w, h] to [x1, y1, x2, y2]
        annotations[:, 2] = annotations[:, 0] + annotations[:, 2]
        annotations[:, 3] = annotations[:, 1] + annotations[:, 3]

        return annotations

    def coco_label_to_label(self, coco_label):
        return self.coco_labels_inverse[coco_label]

    def label_to_coco_label(self, label):
        return self.coco_labels[label]


def collater(data):
    imgs = [s['img'] for s in data]
    annots = [s['annot'] for s in data]
    scales = [s['scale'] for s in data]

    imgs = torch.from_numpy(np.stack(imgs, axis=0))

    max_num_annots = max(annot.shape[0] for annot in annots)

    if max_num_annots > 0:

        annot_padded = torch.ones((len(annots), max_num_annots, 5)) * -1

        if max_num_annots > 0:
            for idx, annot in enumerate(annots):
                if annot.shape[0] > 0:
                    annot_padded[idx, :annot.shape[0], :] = annot
    else:
        annot_padded = torch.ones((len(annots), 1, 5)) * -1

    imgs = imgs.permute(0, 3, 1, 2)

    return {'img': imgs, 'annot': annot_padded, 'scale': scales}


class Resizer(object):
    """Convert ndarrays in sample to Tensors."""
    
    def __init__(self, img_size=512):
        self.img_size = img_size

    def __call__(self, sample):
        image, annots = sample['img'], sample['annot']
        height, width, _ = image.shape
        if height > width:
            scale = self.img_size / height
            resized_height = self.img_size
            resized_width = int(width * scale)
        else:
            scale = self.img_size / width
            resized_height = int(height * scale)
            resized_width = self.img_size

        image = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)

        new_image = np.zeros((self.img_size, self.img_size, 3))
        new_image[0:resized_height, 0:resized_width] = image

        annots[:, :4] *= scale

        return {'img': torch.from_numpy(new_image).to(torch.float32), 'annot': torch.from_numpy(annots), 'scale': scale}


class Augmenter(object):
    """Convert ndarrays in sample to Tensors."""

    def __call__(self, sample, flip_x=0.5):
        if np.random.rand() < flip_x:
            image, annots = sample['img'], sample['annot']
            image = image[:, ::-1, :]

            rows, cols, channels = image.shape

            x1 = annots[:, 0].copy()
            x2 = annots[:, 2].copy()

            x_tmp = x1.copy()

            annots[:, 0] = cols - x2
            annots[:, 2] = cols - x_tmp

            sample = {'img': image, 'annot': annots}

        return sample


class Normalizer(object):

    def __init__(self, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]):
        self.mean = np.array([[mean]])
        self.std = np.array([[std]])

    def __call__(self, sample):
        image, annots = sample['img'], sample['annot']

        return {'img': ((image.astype(np.float32) - self.mean) / self.std), 'annot': annots}
