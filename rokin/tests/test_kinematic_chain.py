from unittest import TestCase
from wzk.time import tic, toc


class Test(TestCase):
    def test_prev2next_frame_idx(self):
        pfi = np.array([-1, 0, 0, 1, 2, 3, 3, 3])
        nfi = prev2next_frame_idx(pfi)
        pfi2 = next2prev_frame_idx(nfi)
        self.assertTrue(all(pfi == pfi2))

    def test_next2prev_frame_idx(self):
        nfi = np.array([1, 2, [3, 10], 4, 5, 6, 7, 8, 9, -1, 11, -1], dtype='object')
        pfi = next2prev_frame_idx(nfi)
        nfi2 = prev2next_frame_idx(pfi)
        self.assertTrue(all(nfi == nfi2))

    def test_combine_chains(self):
        nfi = np.array([1, 2, 3, 4, 5, -1])
        nfi_list = [nfi]*4
        nfi_combined = [1,  2,  3,  4,  5,  -1,
                        7,  8,  9, 10, 11,  -1,
                        13, 14, 15, 16, 17, -1,
                        19, 20, 21, 22, 23, -1]
        nfi_combined2 = combine_chains(nfi_list)

        self.assertTrue(all(nfi_combined == nfi_combined2))

    def test_speed(self):

        next_frame_idx = np.array([1, 2, 3, 4, [5, 14, 23],  # 0-4
                                   6, 7, 8, 9, 10, 11, 12, 13, -1,  # 5-13
                                   15, 16, 17, 18, 19, 20, 21, 22, -1,  # 14-22
                                   24, 25, 26, -1], dtype='object')  # 22-26

        joint_frame_idx = np.array([1, [2, 4], [3, 4],  # 0-2
                                    6, 7, 8, 9, 10, 11, 12,  # 3-10
                                    15, 16, 17, 18, 19, 20, 21,  # 11-17
                                    24, 25], dtype='object')  # 18-19

        n = 100
        prev_frame_idx = None
        tic()
        for i in range(n):
            prev_frame_idx = next2prev_frame_idx(nfi=next_frame_idx)
        toc('prev_frame_idx')

        tic()
        for i in range(n):
            next_frame_idx = prev2next_frame_idx(pfi=prev_frame_idx)
        toc('next_frame_idx')

        tic()
        for i in range(n):
            _ = next_frame_idx2influence_frames_frames(nfi=next_frame_idx)
        toc('INFLUENCE_FRAMES_FRAMES')

        tic()
        for i in range(n):
            _ = influence_frames_frames2joints_frames(nfi=next_frame_idx, jfi=joint_frame_idx)
        toc('joint_frame_influence')

        tic()
        for i in range(n):
            _ = np.ones((10, 4, 4)) @ np.ones((10, 4, 4))
        toc('Matrix Multiplication')
