from src.dir_cleaner import *
import mock
from mock import MagicMock, call, patch

class TestDirCleaner(object):
    ## Tests if both under the limit and over the limit file amounts are handled coorectly
    @patch('src.dir_cleaner.DirCleaner.listdir', return_value = ["a", "b", "c"])
    @patch('src.dir_cleaner.DirCleaner.remove')
    @patch('os.path.getmtime', return_value = 1)
    def testLessAndMoreFilesThanTheLimit(self, a, b, c):
        dc = DirCleaner()

        dc.keep_local_max("/test", 50)
        dc.remove.assert_not_called()

        dc.keep_local_max("/test", 1)
        calls = [call("/test/a"), call("/test/b")]
        dc.remove.assert_has_calls(calls, any_order=False)

    ## Tests if the worker calls the keep_local_max function correctly
    @patch('src.dir_cleaner.DirCleaner.keep_local_max')
    def testWorker(self, a):
        dc = DirCleaner()

        dc.worker("test_path", 999, True)
        dc.keep_local_max.assert_called_once()

    ## Tests if the params are parsed correctly
    def testParams(self):
        args = DirCleaner.parseArgs(["--path", "/test", "--keep_max", "123", "__name", "some name", "__log", "some log"])
        assert args.path == "/test"
        assert args.keep_max == "123"

    class c_args_val:
        pass
    args_val = c_args_val()
    args_val.path = "path"
    args_val.keep_max = 8

    @patch('src.dir_cleaner.DirCleaner.__init__', return_value = None)
    @patch('src.dir_cleaner.DirCleaner.parseArgs', return_value = args_val)
    @patch('src.dir_cleaner.DirCleaner.worker')
    @patch('rospy.init_node')
    def testMain(self, init_node, worker, parse_args, init_dc):
        main()
        init_node.assert_called_with("dir_cleaner", anonymous = True)
        init_dc.assert_called_once()
        worker.assert_called_with('path', 8)
        parse_args.assert_called_once()


