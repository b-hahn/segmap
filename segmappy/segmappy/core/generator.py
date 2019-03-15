from __future__ import print_function
import numpy as np

from ..tools.classifiertools import to_onehot


class Generator(object):
    def __init__(
        self,
        preprocessor,
        segment_ids,
        n_classes,
        train=True,
        batch_size=16,
        shuffle=False,
    ):
        self.preprocessor = preprocessor
        self.segment_ids = segment_ids
        self.n_classes = n_classes
        self.train = train
        self.batch_size = batch_size
        self.shuffle = shuffle

        self.n_segments = len(self.segment_ids)
        self.n_batches = int(np.ceil(float(self.n_segments) / batch_size))

        self._i = 0

    def __iter__(self):
        return self

    def next(self):
        if self.shuffle and self._i == 0:
            np.random.shuffle(self.segment_ids)

        self.batch_ids = self.segment_ids[self._i : self._i + self.batch_size]

        self._i = self._i + self.batch_size
        if self._i >= self.n_segments:
            self._i = 0
        # TODO: either here or in segmappy_train_cnn, add option to dynamically feed either occ grid, color, semantics o rcolor + semantics
        batch_segments, batch_classes, batch_semantic_classes = self.preprocessor.get_processed(
            self.batch_ids, train=self.train
        )

        # TODO: can I leave this out? Why was adding an extra dimension previously needed?
        # batch_segments = batch_segments[:, :, :, :, None]
        # print("batch_segments.shape: ", batch_segments.shape)
        batch_classes = to_onehot(batch_classes, self.n_classes)

        # convert semantic class integer for each point in segment to (65,) vector containing counts of each class per segment
        # segment_semantic_class_distribution = np.zeros((len(batch_segments), self.preprocessor.n_semantic_classes))
        # for i in range(len(batch_segments)):
        #     ss = np.bincount(np.int64(batch_segments[i, :, :, :, :].flatten()))
        #     print("ss: ", ss)
        #     # segment_semantic_class_distribution[i, batch_segments[i, :, :, :, all of these]]
        # print("batch_segments.shape:", batch_segments.shape)

        # remove semantic class from batch_segments array

        return batch_segments, batch_classes, batch_semantic_classes


class GeneratorFeatures(object):
    def __init__(self, features, classes, n_classes=2, batch_size=16, shuffle=True):
        self.features = features
        self.classes = np.asarray(classes)
        self.n_classes = n_classes
        self.batch_size = batch_size
        self.shuffle = shuffle
        self.n_samples = features.shape[0]
        self.n_batches = int(np.ceil(float(self.n_samples) / batch_size))
        self._i = 0

        self.sample_ids = list(range(self.n_samples))
        if shuffle:
            np.random.shuffle(self.sample_ids)

    def next(self):
        batch_ids = self.sample_ids[self._i : self._i + self.batch_size]

        self._i = self._i + self.batch_size
        if self._i >= self.n_samples:
            self._i = 0

        batch_features = self.features[batch_ids, :]
        batch_classes = self.classes[batch_ids]
        batch_classes = to_onehot(batch_classes, self.n_classes)

        return batch_features, batch_classes
