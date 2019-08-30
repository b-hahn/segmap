from __future__ import print_function
import numpy as np
import itertools

class Preprocessor(object):
    def __init__(
        self,
        augment_angle=0.0,
        augment_remove_random_min=0.0,
        augment_remove_random_max=0.0,
        augment_remove_plane_min=0.0,
        augment_remove_plane_max=0.0,
        augment_jitter=0.0,
        align="none",
        voxelize=True,
        scale_method="fixed",
        center_method="none",
        scale=(1, 1, 1),
        voxels=(1, 1, 1),
        remove_mean=False,
        remove_std=False,
        batch_size=16,
        scaler_train_passes=1,
    ):
        self.augment_remove_random_min = augment_remove_random_min
        self.augment_remove_random_max = augment_remove_random_max
        self.augment_remove_plane_min = augment_remove_plane_min
        self.augment_remove_plane_max = augment_remove_plane_max
        self.augment_angle = augment_angle
        self.augment_jitter = augment_jitter

        self.align = align
        self.voxelize = voxelize
        self.scale_method = scale_method
        self.center_method = center_method
        self.scale = np.array(scale)
        self.voxels = np.array(voxels)
        self.remove_mean = remove_mean
        self.remove_std = remove_std
        self.batch_size = batch_size
        self.scaler_train_passes = scaler_train_passes
        self._scaler_exists = False

        self.n_semantic_classes = 66  # in mapillary vistas dataset

        min_voxel_side_length_m = 0.1
        self.min_scale = self.voxels * min_voxel_side_length_m

        self.last_scales = []

    def init_segments(self,
                      segments,
                      classes,
                      positions=None,
                      train_ids=None,
                      scaler_path=None,
                      segments_colors=None,
                      segments_semantic_classes=None):

        self.segments = segments
        self.classes = np.array(classes)

        if segments_colors != None:
            self.use_segment_colors = True
            self.segments_colors = segments_colors
        else:
            self.use_segment_colors = False
        # print("segments_colors: {}, self.use_segment_colors: {}".format(segments_colors, self.use_segment_colors))


        if segments_semantic_classes != None:
            self.use_segment_semantics = True
            self.segments_semantic_classes = segments_semantic_classes
        else:
            self.use_segment_semantics = False

        if self.align == "robot":
            assert positions is not None
            self.segments = self._align_robot(self.segments, positions)

        # check if we need to train a scaler
        if self.remove_mean or self.remove_std:
            if scaler_path is None:
                assert train_ids is not None
                self._train_scaler(train_ids)
            else:
                self.load_scaler(scaler_path)

    def get_processed(self, segment_ids, train=True, normalize=True):
        batch_segments = []
        batch_segments_colors = []
        batch_segments_semantic_classes = []
        for i in segment_ids:
            batch_segments.append(self.segments[i])
            if self.use_segment_colors:
                batch_segments_colors.append(self.segments_colors[i])
            else:
                batch_segments_colors = []
            if self.use_segment_semantics:
                batch_segments_semantic_classes.append(
                    self.segments_semantic_classes[i])
            else:
                batch_segments_semantic_classes = []

        batch_segments, batch_semantic_classes = self.process(
            batch_segments, train, normalize, batch_segments_colors,
            batch_segments_semantic_classes)
        batch_classes = self.classes[segment_ids]

        return batch_segments, batch_classes, batch_semantic_classes

    def process(self, segments, train=True, normalize=True, segments_colors=[], segments_semantic_classes=[]):
        # augment through distorsions
        if train and self.augment_remove_random_max > 0:
            segments, segments_colors, segments_semantic_classes = self._augment_remove_random(
                segments, segments_colors, segments_semantic_classes)

        if train and self.augment_remove_plane_max > 0:
            segments, segments_colors, segments_semantic_classes = self._augment_remove_plane(
                segments, segments_colors, segments_semantic_classes)

        # align after distorsions
        if self.align == "eigen":
            segments = self._align_eigen(segments)

        # augment rotation
        if train and self.augment_angle > 0:
            segments = self._augment_rotation(segments)

        if self.voxelize:
            # rescale coordinates and center
            segments = self._rescale_coordinates(segments)

            # randomly displace the segment
            if train and self.augment_jitter > 0:
                segments = self._augment_jitter(segments)

            # insert into voxel grid
            if self.use_segment_colors and self.use_segment_semantics:
                segments, segments_semantic_classes = self._voxelize_color_semantics(segments, segments_colors, segments_semantic_classes)
            else:
                segments = self._voxelize(segments)

            # remove mean and/or std
            if normalize and self._scaler_exists:
                segments = self._normalize_voxel_matrix(segments)

        return segments, segments_semantic_classes

    def get_n_batches(self, train=True):
        if train:
            return self.n_batches_train
        else:
            return self.n_batches_test

    # create rotation matrix that rotates point around
    # the origin by an angle theta, expressed in radians
    def _get_rotation_matrix_z(self, theta):
        R_z = [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ]

        return np.array(R_z)

    # align according to the robot's position
    def _align_robot(self, segments, positions):
        aligned_segments = []
        for i, seg in enumerate(segments):
            center = np.mean(seg, axis=0)

            robot_pos = positions[i] - center
            seg = seg - center

            # angle between robot and x-axis
            angle = np.arctan2(robot_pos[1], robot_pos[0])

            # align the segment so the robots perspective is along the x-axis
            inv_rotation_matrix_z = self._get_rotation_matrix_z(angle)
            aligned_seg = np.dot(seg, inv_rotation_matrix_z)

            aligned_segments.append(aligned_seg)

        return aligned_segments

    def _align_eigen(self, segments):
        aligned_segments = []
        for segment in segments:
            # Calculate covariance
            center = np.mean(segment, axis=0)

            covariance_2d = np.cov(segment[:, :2] - center[:2], rowvar=False, bias=True)

            eigenvalues, eigenvectors = np.linalg.eig(covariance_2d)
            alignment_rad = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])

            if eigenvalues[0] < eigenvalues[1]:
                alignment_rad = alignment_rad + np.pi / 2

            inv_rotation_matrix_z = self._get_rotation_matrix_z(alignment_rad)
            aligned_segment = np.dot(segment, inv_rotation_matrix_z)

            y_center = np.mean(segment[:, 1])
            n_below = np.sum(segment[:, 1] < y_center)

            if n_below < segment.shape[0] / 2:
                alignment_rad = alignment_rad + np.pi
                inv_rotation_matrix_z = self._get_rotation_matrix_z(np.pi)
                aligned_segment = np.dot(aligned_segment, inv_rotation_matrix_z)

            aligned_segments.append(aligned_segment)

        return aligned_segments

    # augment with multiple rotation of the same segment
    def _augment_rotation(self, segments):
        angle_rad = self.augment_angle * np.pi / 180

        augmented_segments = []
        for segment in segments:
            rotation = np.random.uniform(-angle_rad, angle_rad)
            segment = np.dot(segment, self._get_rotation_matrix_z(rotation))
            augmented_segments.append(segment)

        return augmented_segments

    # TODO(ben): add proper checks to see if segments_colors is being used
    def _augment_remove_random(self, segments, segments_colors=[], segments_semantic_classes=[]):
        augmented_segments = []
        augmented_segments_colors = []
        augmented_segment_semantic_classes = []
        for segment, segment_color, segment_semantic_class in itertools.zip_longest(segments, segments_colors, segments_semantic_classes):
            # percentage of points to remove
            remove = (
                np.random.random()
                * (self.augment_remove_random_max - self.augment_remove_random_min)
                + self.augment_remove_random_min
            )

            # randomly choose the points
            idx = np.arange(segment.shape[0])
            np.random.shuffle(idx)
            idx = idx[int(idx.size * remove) :]

            segment = segment[idx]
            augmented_segments.append(segment)

            if segments_colors != []:
                segment_color = segment_color[idx]
                augmented_segments_colors.append(segment_color)
            if segments_semantic_classes != []:
                segment_semantic_class = segment_semantic_class[idx]
                augmented_segment_semantic_classes.append(segment_semantic_class)

        return augmented_segments, augmented_segments_colors, augmented_segment_semantic_classes

    def _augment_remove_plane(self, segments, segments_colors=[], segments_semantic_classes=[]):
        augmented_segments = []
        augmented_segments_colors = []
        augmented_segments_semantic_classes = []
        for segment, segment_color, segment_semantic_classes in itertools.zip_longest(
                segments, segments_colors, segments_semantic_classes):
            # center segment
            center = np.mean(segment, axis=0)
            segment = segment - center

            # slice off a section of the segment
            while True:
                # generate random plane
                plane_norm = np.random.random(3) - 0.5
                plane_norm = plane_norm / np.sqrt(np.sum(plane_norm ** 2))

                # on which side of the plane each point is
                sign = np.dot(segment, plane_norm)

                # find an offset that removes a desired amount of points
                found = False
                plane_offsets = np.linspace(
                    -np.max(self.scale), np.max(self.scale), 100
                )
                np.random.shuffle(plane_offsets)
                for plane_offset in plane_offsets:
                    keep = sign + plane_offset > 0
                    remove_percentage = 1 - (np.sum(keep) / float(keep.size))

                    if (
                        remove_percentage > self.augment_remove_plane_min
                        and remove_percentage < self.augment_remove_plane_max
                    ):
                        segment = segment[keep]
                        if segments_colors != []:
                            segment_color = segment_color[keep]
                        if segments_semantic_classes != []:
                            segment_semantic_classes = segment_semantic_classes[keep]
                        found = True
                        break

                if found:
                    break

            segment = segment + center
            augmented_segments.append(segment)
            if segments_colors != []:
                augmented_segments_colors.append(segment_color)
            if segment_semantic_classes != []:
                augmented_segments_semantic_classes.append(segment_semantic_classes)

        return augmented_segments, augmented_segments_colors, augmented_segments_semantic_classes

    def _augment_jitter(self, segments):
        jitter_segments = []
        for segment in segments:
            jitter = np.random.random(3) * 2 - 1
            jitter = jitter * self.augment_jitter * self.voxels / 2

            segment = segment + jitter
            jitter_segments.append(segment)

        return jitter_segments

    def _rescale_coordinates(self, segments):
        # center corner to origin
        centered_segments = []
        for segment in segments:
            segment = segment - np.min(segment, axis=0)
            centered_segments.append(segment)
        segments = centered_segments

        # store the last scaling factors that were used
        self.last_scales = []

        # rescale coordinates to fit inside voxel matrix
        rescaled_segments = []
        for segment in segments:
            # choose scale
            if self.scale_method == "fixed":
                scale = self.scale
                segment = segment / scale * (self.voxels - 1)
            elif self.scale_method == "aspect":
                scale = np.tile(np.max(segment), 3)
                segment = segment / scale * (self.voxels - 1)
            elif self.scale_method == "fit":
                scale = np.max(segment, axis=0)
                thresholded_scale = np.maximum(scale, self.min_scale)
                segment = segment / thresholded_scale * (self.voxels - 1)

            # recenter segment
            if self.center_method != "none":
                if self.center_method == "mean":
                    center = np.mean(segment, axis=0)
                elif self.center_method == "min_max":
                    center = np.max(segment, axis=0) / 2.0

                segment = segment + (self.voxels - 1) / 2.0 - center

            self.last_scales.append(scale)
            rescaled_segments.append(segment)

        return rescaled_segments

    def _voxelize_color_semantics(self, segments, segments_colors, segments_semantic_class):
        # voxel grid contains (bool occupied, int r, int g, int b, int semantic_class) at each position in space for each segment
        # n_semantic_classes + 1 corresponds to empty space
        voxelized_segments = np.zeros((len(segments),) + tuple(self.voxels) + (4,))
        voxelized_segments[:, :, :, :, 0] = (self.n_semantic_classes + 1)  # set occupancy values to value representing empty voxel
        voxelized_segments_semantic_classes = np.zeros((len(segments), self.n_semantic_classes))
        voxel_color_counter = np.zeros(tuple(self.voxels))
        for i, (segment, segment_color, segment_semantic_class) in enumerate(
                zip(segments, segments_colors, segments_semantic_class)):
            # remove out of bounds points
            oob_idx1 = np.all(segment < self.voxels, axis=1)
            segment = segment[oob_idx1, :]
            oob_idx2 = np.all(segment >= 0, axis=1)
            segment = segment[oob_idx2, :]
            segment_color = segment_color[oob_idx1, :]
            segment_color = segment_color[oob_idx2, :]
            # TODO: semantic class is stored as integer (0-65 for mapillary vistas). Change either to one-hot somewhere, and/or
            # accumulate the number of classes for each point in a segment
            segment_semantic_class = segment_semantic_class[oob_idx1]
            segment_semantic_class = segment_semantic_class[oob_idx2]

            # round coordinates
            segment = segment.astype(np.int)

            # fill voxel grid
            #TODO: this could be done in a more efficient manner

            # check if this voxel has been previously filled already; if so, calculate mean color)
            for j in range(len(segment)):
                voxelized_segments[i, segment[j, 0], segment[j, 1],
                                    segment[j, 2]] += np.hstack(
                                        [0, segment_color[j]])
                voxel_color_counter[segment[j, 0], segment[j, 1],
                                    segment[j, 2]] += 1
                voxelized_segments_semantic_classes[i, int(segment_semantic_class[j])] += 1
            # print("----------> segment_semantic_class: ", segment_semantic_class)
            # print("----------> voxelized_segments_semantic_classes[i, :]: ", voxelized_segments_semantic_classes[i, :])

            # compute mean color
            voxelized_segments[i, segment[:, 0], segment[:, 1], segment[:, 2], 1:] /= voxel_color_counter[segment[:, 0], segment[:, 1], segment[:, 2]][:, np.newaxis]
            if np.max(voxelized_segments[i, segment[:, 0], segment[:, 1], segment[:, 2]]) > 255:
                # TODO: verify this never occurs
                print("Error: max color value > 255!!")
                exit(0)

        return voxelized_segments, voxelized_segments_semantic_classes

    def _voxelize(self, segments):
        voxelized_segments = np.zeros((len(segments),) + tuple(self.voxels) + (1,))
        for i, segment in enumerate(segments):
            # remove out of bounds points
            segment = segment[np.all(segment < self.voxels, axis=1), :]
            segment = segment[np.all(segment >= 0, axis=1), :]

            # round coordinates
            segment = segment.astype(np.int)

            # fill voxel grid
            voxelized_segments[i, segment[:, 0], segment[:, 1], segment[:, 2], 0] = 1

        return voxelized_segments

    def _train_scaler(self, train_ids):
        from sklearn.preprocessing import StandardScaler

        scaler = StandardScaler(with_mean=self.remove_mean, with_std=self.remove_std)

        for p in range(self.scaler_train_passes):
            for i in np.arange(0, len(train_ids), self.batch_size):
                segment_ids = train_ids[i : i + self.batch_size]
                segments, _ = self.get_processed(segment_ids)
                segments = np.reshape(segments, (segments.shape[0], -1))
                scaler.partial_fit(segments)

        self._scaler = scaler
        self._scaler_exists = True

    # remove mean and std
    def _normalize_voxel_matrix(self, segments):
        segments = np.reshape(segments, (segments.shape[0], -1))
        segments = self._scaler.transform(segments)
        segments = np.reshape(segments, (segments.shape[0],) + tuple(self.voxels))

        return segments

    def save_scaler(self, path):
        import pickle

        with open(path, "w") as fp:
            pickle.dump(self._scaler, fp)

    def load_scaler(self, path):
        import pickle

        with open(path, "r") as fp:
            self._scaler = pickle.load(fp)
            self._scaler_exists = True
