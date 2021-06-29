import unittest
import platform

from rokin.forward import *
from rokin.util import str2robot

from wzk import (numeric_derivative, tic, toc,
                 new_fig, save_fig,
                 compare_arrays)


if platform.system() == 'Linux':
    from rokin.Robots.Justin19_wolfram import justin19 as justin19_wolfram
else:
    justin19_wolfram = None

robot_ids = ['JustinHand12', 'SingleSphere02', 'SingleSphere03',
             'StaticArm02', 'StaticArm07',
             'MovingArm01', 'MovingArm06',
             'Justin19', 'JustinArm07',
             'JustinBase03', 'JustinBase05',
             'JustinFinger03',
             'JustinHand12']

eps = 1e-5
verbose = 5


class Test(unittest.TestCase):

    def __test_get_frames_jac(self, robot_id):
        robot = str2robot(robot_id)
        q = sample_q(robot=robot, mode='random')
        print(robot_id)
        grad_numeric = numeric_derivative(fun=get_frames, x=q, robot=robot,
                                          axis=-1, eps=eps)
        grad_analytic = get_frames_jac(q=q, robot=robot)[1]
        # d = compare_arrays(grad_numeric, grad_analytic, axis=(-4, -1), verbose=verbose-1)
        # d1 = compare_arrays(grad_numeric*2, grad_analytic, axis=(-4, -1), verbose=verbose-1)
        print(np.round(grad_analytic[0, :10, :, :, 0], 2))
        print(np.round(grad_numeric[0, :10, :, :, 0], 2))
        self.assertTrue(compare_arrays(a=grad_numeric, b=grad_analytic, axis=(-4, -1),
                                       title='Frames ' + robot.id, verbose=verbose-1))

    def __test_get_x_spheres_jac(self, robot_id):
        robot = str2robot(robot_id)
        q = sample_q(robot=robot, mode='random2')

        grad_numeric = numeric_derivative(fun=get_x_spheres, x=q, axis=-1, eps=eps, robot=robot)
        grad_analytic = get_x_spheres_jac(q=q, robot=robot)[1]
        self.assertTrue(compare_arrays(a=grad_numeric, b=grad_analytic, axis=(-3, -1),
                                       title='Spheres ' + robot.id, verbose=verbose-1))

    def test_get_frames_jac(self):
        for robot_id in robot_ids:
            self.__test_get_frames_jac(robot_id=robot_id)

    def test_get_x_spheres_jac(self):
        for robot_id in robot_ids:
            self.__test_get_x_spheres_jac(robot_id=robot_id)

    def test_justin_cython(self):

        if justin19_wolfram is None:
            print('Test excluded: FullBody_19 Numpy <-> Cython (Original)')
            self.assertTrue(True)
            return

        robot = str2robot('Justin19')
        q = sample_q(robot=robot, mode='random')

        robot.f_world_robot = None
        frames_cython = justin19_wolfram.get_frames(q=q, f_world_robot=robot.f_world_robot)
        frames = get_frames(q=q, robot=robot)

        self.assertTrue(compare_arrays(a=frames_cython, b=frames, axis=(2,), title='Justin Cython Frames'))

        frames_jac_cython = justin19_wolfram.get_frames_jac(q=q, f_world_robot=robot.f_world_robot)[1]
        frames_jac = get_frames_jac(q=q, robot=robot)[1]
        print(frames_jac_cython[0, 0, 1, 18])
        self.assertTrue(compare_arrays(a=frames_jac_cython, b=frames_jac, axis=(2, 3),
                                       title='Justin Cython Frames Jac'))

    @staticmethod
    def generate_justin_kinematic_ground_truth(n=10000):
        q = np.random.uniform(-np.pi, np.pi, (n, 1, 19))
        q[0] = 0
        f, j = justin19_wolfram.get_frames_jac(q=q[:n], f_world_robot=None)
        np.save('justin_kinematic_ground_truth.npy', (q, f, j, []))

    def test_justin_speed(self):

        assert_equal = False
        if justin19_wolfram is None:
            print('Test excluded: FullBody_19 Numpy <-> Cython (Original)')
            self.assertTrue(True)
            return

        m = 10
        n_samples_list = np.unique(np.logspace(0, 4, num=100).astype(int))

        robot = str2robot('Justin19')
        q = sample_q(robot=robot, mode=int(max(n_samples_list)))
        robot.f_world_robot = None

        f, f_cython = None, None
        j, j_cython = None, None
        time_frames_cython, time_frames_cpp, time_frames_numpy = [], [], []
        time_jac_cython, time_jac_cpp, time_jac_numpy = [], [], []

        for n in n_samples_list:
            tic()
            for _ in range(m):
                f_cython = justin19_wolfram.get_frames(q=q[:n], f_world_robot=robot.f_world_robot)
            time_frames_cython.append(toc(name=''))
            tic()
            for _ in range(m):
                f = get_frames(q=q[:n], robot=robot)
            time_frames_numpy.append(toc(name=''))
            tic()
            for _ in range(m):
                j_cython = justin19_wolfram.get_frames_jac(q=q[:n], f_world_robot=robot.f_world_robot)[1]
            time_jac_cython.append(toc(name=''))
            tic()
            for _ in range(m):
                j = get_frames_jac(q=q[:n], robot=robot)[1]
            time_jac_numpy.append(toc(name=''))

            if assert_equal:
                self.assertTrue(compare_arrays(a=f_cython, b=f, title='Justin Cython Frames', axis=(2,)))
                self.assertTrue(compare_arrays(a=j_cython, b=j, title='Justin Cython Frames Jac', axis=(2, 3)))

        if verbose >= 1:
            (time_frames_cython, time_frames_cpp, time_frames_numpy,
             time_jac_cython, time_jac_cpp, time_jac_numpy) = \
                np.atleast_1d(time_frames_cython, time_frames_cpp, time_frames_numpy,
                              time_jac_cython, time_jac_cpp, time_jac_numpy)

            fig, ax = new_fig(title='Justin Speed Comparison', scale=2)
            ax.semilogx(n_samples_list, time_frames_cython / time_frames_numpy, c='b', marker='x',
                        label='frame cython/numpy')
            ax.semilogx(n_samples_list, time_frames_cython / time_frames_cpp, c='b', marker='o',
                        label='frame cython/bullet')
            ax.semilogx(n_samples_list, time_jac_cython / time_jac_numpy, c='r', marker='x',
                        label='jac cython/numpy')
            ax.semilogx(n_samples_list, time_jac_cython / time_jac_cpp, c='r', marker='o',
                        label='jac cython/bullet')
            ax.legend()
            save_fig(fig=fig)
        res = np.stack([n_samples_list, time_frames_cython, time_frames_numpy, time_jac_cython, time_jac_numpy]).T
        # [[     1.              0.00007963      0.00030255      0.0001049       0.00075769]
        #  [     2.              0.00004435      0.00024462      0.00015306      0.00082469]
        #  [     3.              0.0000453       0.00023723      0.000175        0.0008831 ]
        #  [     4.              0.00004983      0.00023937      0.00020242      0.00094438]
        #  [     5.              0.00005651      0.00026178      0.00025225      0.00104547]
        #  [     6.              0.00006962      0.00025582      0.0001719       0.00114799]
        #  [     7.              0.00006962      0.00025654      0.00029087      0.00123096]
        #  [     8.              0.00008583      0.00026393      0.00022078      0.0013206 ]
        #  [     9.              0.00008345      0.00027394      0.0003581       0.00135016]
        #  [    10.              0.00008821      0.00028515      0.0004642       0.00149274]
        #  [    11.              0.00009704      0.00033164      0.00028849      0.00151825]
        #  [    12.              0.00010252      0.00028801      0.00039649      0.00160742]
        #  [    14.              0.00011158      0.00029516      0.0003407       0.00177073]
        #  [    16.              0.00012422      0.00033998      0.0005815       0.00203538]
        #  [    18.              0.00014186      0.00033164      0.00080562      0.00210118]
        #  [    20.              0.00014997      0.00035405      0.00053692      0.00245261]
        #  [    23.              0.00017452      0.00036526      0.00094962      0.00261235]
        #  [    25.              0.00018787      0.0003767       0.00105         0.00326633]
        #  [    29.              0.00021482      0.00039053      0.00128722      0.00326848]
        #  [    32.              0.00023413      0.00042915      0.00081754      0.00395584]
        #  [    36.              0.00025892      0.00044203      0.00150204      0.00376987]
        #  [    41.              0.0002923       0.00048351      0.00131416      0.00508213]
        #  [    46.              0.00033784      0.00057173      0.00145555      0.00469184]
        #  [    52.              0.00035572      0.00053692      0.00121021      0.0056324 ]
        #  [    58.              0.00040174      0.00056958      0.00184655      0.00559592]
        #  [    65.              0.00045991      0.00065994      0.00184035      0.00750756]
        #  [    73.              0.00050879      0.00070548      0.00245309      0.00684738]
        #  [    83.              0.00055957      0.00074768      0.00195813      0.00888944]
        #  [    93.              0.00062943      0.00080085      0.00237989      0.00887132]
        #  [   104.              0.00069499      0.00090957      0.00324178      0.01077962]
        #  [   117.              0.00078654      0.00099611      0.00314379      0.01095033]
        #  [   132.              0.00087976      0.00105762      0.00297928      0.01339245]
        #  [   148.              0.00099397      0.00118899      0.00372124      0.01382422]
        #  [   166.              0.00111103      0.00135779      0.00443029      0.01706362]
        #  [   187.              0.00122023      0.00145698      0.00493097      0.01710558]
        #  [   210.              0.00138521      0.0016098       0.00480533      0.02098632]
        #  [   236.              0.00157166      0.00184512      0.00585961      0.02176237]
        #  [   265.              0.00177574      0.00204587      0.00671363      0.02684331]
        #  [   298.              0.00187564      0.00229216      0.00753593      0.02736545]
        #  [   335.              0.00224686      0.00247717      0.00786114      0.03356338]
        #  [   376.              0.00234008      0.00275111      0.009238        0.03472757]
        #  [   422.              0.00274658      0.00309944      0.01021504      0.04257488]
        #  [   475.              0.00299072      0.00346065      0.01186419      0.04352784]
        #  [   533.              0.00336194      0.00387692      0.01272345      0.05305195]
        #  [   599.              0.00393081      0.00429416      0.01415372      0.05821228]
        #  [   673.              0.00430751      0.00473237      0.01804805      0.06869197]
        #  [   756.              0.0047636       0.00722337      0.01894546      0.07836056]
        #  [   849.              0.00533986      0.00609732      0.01998758      0.07994628]
        #  [   954.              0.0059917       0.0067234       0.02281904      0.0907588 ]
        #  [  1072.              0.00668764      0.00764155      0.02545357      0.10129523]
        #  [  1204.              0.00760961      0.00859165      0.02908421      0.11557269]
        #  [  1353.              0.00842905      0.00993848      0.03238177      0.12867284]
        #  [  1519.              0.0095706       0.01128817      0.03577113      0.14497256]
        #  [  1707.              0.01095796      0.01250386      0.04021001      0.16435885]
        #  [  1917.              0.01456285      0.01528978      0.04518437      0.19733644]
        #  [  2154.              0.01453853      0.01792955      0.05034685      0.22398925]
        #  [  2420.              0.01519847      0.01891184      0.05724764      0.24884892]
        #  [  2718.              0.01735139      0.02522516      0.06333375      0.28977871]
        #  [  3053.              0.01929665      0.030761        0.07231522      0.31295419]
        #  [  3430.              0.0229497       0.03372836      0.07975554      0.3591702 ]
        #  [  3853.              0.02468991      0.03495336      0.09052348      0.39370084]
        #  [  4328.              0.02806115      0.0415771       0.10079336      0.48009872]
        #  [  4862.              0.0307312       0.04540396      0.11315608      0.50009227]
        #  [  5462.              0.03554726      0.05558777      0.12684441      0.56470108]
        #  [  6135.              0.03888297      0.05846834      0.14284205      0.62573218]
        #  [  6892.              0.04430246      0.07010365      0.15993929      0.71583891]
        #  [  7742.              0.04917383      0.07355475      0.18026567      0.78967905]
        #  [  8697.              0.05472517      0.08768153      0.20085573      0.88894391]
        #  [  9770.              0.06096125      0.09954166      0.22813082      1.01651359]
        #  [ 10974.              0.0693903       0.11117649      0.25646234      1.11810756]
        #  [ 12328.              0.07690549      0.12376261      0.28633904      1.28374457]
        #  [ 13848.              0.08707571      0.13977695      0.32111955      1.43590546]
        #  [ 15556.              0.09604883      0.15692306      0.36101198      1.61301541]
        #  [ 17475.              0.10908222      0.17911863      0.4048264       1.81615949]
        #  [ 19630.              0.12323022      0.20339417      0.45519352      2.04758978]
        #  [ 22051.              0.14007187      0.23042846      0.50957298      2.29619455]
        #  [ 24770.              0.15408897      0.25846243      0.57315207      2.58930898]
        #  [ 27825.              0.1730473       0.28911686      0.64246869      2.90691876]
        #  [ 31257.              0.19534969      0.32937407      0.71954107      3.2723856 ]
        #  [ 35111.              0.21977282      0.3762815       0.81184769      3.68307781]
        #  [ 39442.              0.24814868      0.4287045       0.8780036       4.17835808]
        #  [ 44306.              0.27989292      0.49856234      0.98402143      4.68937016]
        #  [ 49770.              0.30743766      0.5406208       1.09429836      5.21964407]
        #  [ 55908.              0.3469708       0.61316705      1.23230386      5.97859597]
        #  [ 62802.              0.39447355      0.71557045      1.42298222      6.68688035]
        #  [ 70548.              0.44767857      0.79681778      1.56290793      7.47212315]
        #  [ 79248.              0.49822307      0.89181542      1.76416302      8.36650562]
        #  [ 89021.              0.55204177      1.00440097      1.9597044       9.42320132]
        #  [100000.              0.62161016      1.14005017      2.21888185     10.62638211]]
        print(res)

    def test_Justin19_compare_to_ground_truth(self):
        if justin19_wolfram is None:
            print('Test excluded: FullBody_19 Numpy <-> Cython (Original)')
            self.assertTrue(True)
            return

        robot = str2robot('Justin19')

        q, f, j, _ = np.load('justin_kinematic_ground_truth.npy', allow_pickle=True)

        dh4 = robot.dh
        dh5 = np.zeros((len(q), 20, 5))
        dh5[:, :, :4] = dh4[np.newaxis, :, :]

        f1, j1 = robot.get_frames_jac(q=q)
        f2, j2 = robot.get_frames_jac_dh(q=q, dh=dh4)
        f3, j3 = robot.get_frames_jac_dh(q=q, dh=dh5)

        atol = 1e-5
        assert np.allclose(f, f1, atol=atol)
        assert np.allclose(f, f2, atol=atol)
        assert np.allclose(f, f3, atol=atol)

        assert np.allclose(f1, f2, atol=atol)
        assert np.allclose(f1, f3, atol=atol)

        assert np.allclose(j, j1, atol=atol)
        assert np.allclose(j, j2, atol=atol)
        assert np.allclose(j, j3, atol=atol)


def sample_q(robot, mode):

    if isinstance(mode, int):
        shape = mode

    elif mode == 'random':
        shape = 100

    elif mode == 'random2':
        shape = tuple(np.random.randint(1, 10) for _ in range(np.random.randint(1, 5)))

    elif mode == 'seed0':
        np.random.seed(0)
        shape = 100
    else:
        raise ValueError(f"Unknown mode {mode}")

    q = robot.sample_q(shape)
    return q
