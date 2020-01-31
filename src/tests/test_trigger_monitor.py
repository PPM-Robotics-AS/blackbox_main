from src.trigger_monitor import TriggerMonitor
import mock
from mock import MagicMock, call, patch
import socket
import ftplib
from blackbox_main.srv import UploadResponse, UploadRequest
from blackbox_control.srv import GetAvailableRangeResponse, ExportResponse
from blackbox_main.msg import UploadStatusList, UploadStatus
from rospy import Publisher
from src.ros_node_base import RosNodeBase
import datetime
import time

def is_shutdown():
    return TestTriggerMonitor.is_shutdown <= 0

def rate_sleep():
    TestTriggerMonitor.is_shutdown -= 1

def patched_get_param_missing(key, def_val):
    if key == TestTriggerMonitor.missing_key:
        return def_val
    else:
        return patched_get_param(key, def_val)

def patched_get_param(key, def_val):
    if def_val == None:
        if key == "/ns/conditions":
            return "{}"
        else:
            return '{0}:{1}'.format(key, def_val)
    else:
        return def_val

def get_available_range_response():
    ret = GetAvailableRangeResponse()
    return ret

def get_export_response(a,b):
    er = ExportResponse()
    er.file_name = "test"
    return er


class TestTriggerMonitor(object):
    ## Tests if the parameters have the right default value
    def testDefaults(self):
        tm = TriggerMonitor()
        assert tm.node_id == "BBTM"
        assert tm.conditions == None
        assert tm.exporting == False
        assert tm.prev_values == {}
        assert tm.prev_change == {}

    ## Tests if the formatted log messages are emitted
    @patch('src.ros_node_base.RosNodeBase._formatted_log')
    def testFormattedLog(self, formatted_log):
        tm = TriggerMonitor()
        tm.formatted_log('i', 'test')
        formatted_log.assert_called_once()
        formatted_log.assert_called_with(tm, 'i', 'test', 'BBTM')
        formatted_log.reset_mock()
        tm.formatted_log('w', 'test2')
        formatted_log.assert_called_once()
        formatted_log.assert_called_with(tm, 'w', 'test2', 'BBTM')
        formatted_log.reset_mock()
        tm.formatted_log('e', 'test3')
        formatted_log.assert_called_once()
        formatted_log.assert_called_with(tm, 'e', 'test3', 'BBTM')

## Tests if the currect namespace is used
    @patch('rospy.get_name', return_value="xxx")
    def testNs(self, get_name):
        tm = TriggerMonitor()
        assert tm._ns("test") == "xxx/test"
        assert tm._ns("test2") == "xxx/test2"
        assert tm._ns("test3") == "xxx/test3"

    

    def patched_ns(self, key):
        return "/ns/" + key

    ## Tests if the config is loaded correctly
    @patch('rospy.logerr')
    @patch('rospy.logwarn')
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testLoadConfig(self, ns, get_param, logwarn, logerr):
        tm = TriggerMonitor()
        tm.load_config()
        assert tm.conditions == {}
        assert tm.on_change_only == False
        assert tm.trigger_delay == 1000
        assert tm.bag_from == 5
        assert tm.bag_to == 5
        assert tm.bag_path == '/ns/bag_path:None'

        logwarn.assert_not_called()
        logerr.assert_not_called()
    
    ## The missing key for testing mandatory and recommended keys missing
    missing_key = None


    ## Tests if the missing recommended config items are handled correctly
    @patch('sys.exit')
    @patch('rospy.logerr')
    @patch('rospy.logwarn')
    @patch('rospy.get_param', side_effect=patched_get_param_missing)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testLoadConfig_RecommendedMissing(self, ns, get_param, logwarn, logerr, _exit):
        tm = TriggerMonitor()
        tm.load_config()

        logwarn.assert_not_called()
        logerr.assert_not_called()
        _exit.assert_not_called()

        logwarn.reset_mock()
        TestTriggerMonitor.missing_key = "/ns/bag_path"
        tm.load_config()

        calls = [call('[BBTM] Mandatory config item not set: bag_path'), call('[BBTM] Mandatory config item(s) missing, exiting.')]
        logerr.assert_has_calls(calls)
        logwarn.assert_not_called()
        _exit.assert_called_once()

    ## Tests if the formatted log messages are emitted
    @patch('rospy.loginfo')
    @patch('rospy.logwarn')
    @patch('rospy.logerr')
    def _testFormattedLog(self, logerr, logwarn, loginfo):
        #cu = CloudUploader()
        #cu.formatted_log('i', "test")
        #loginfo.assert_called_with("[BBU] test")
        pass


    ## Tests if a new prev change is added
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testNoPrevChange(self, get_param, ns):
        tm = TriggerMonitor()
        tm.load_config()
        assert tm.exporting == False
        assert tm.on_change_only == False
        
        tm.variable_changed("test", "var1")
        tm.variable_changed("test", "var2")
        assert tm.prev_values["var1"] == "test"
        assert tm.prev_values["var2"] == "test"
        assert tm.prev_change["var1"] != None
        assert tm.prev_change["var2"] != None
        assert len(tm.prev_values.items()) == 2

    ## Tests if a new prev change is not added if already exporting, but prev value is
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testNoPrevChange(self, get_param, ns):
        tm = TriggerMonitor()
        tm.load_config()
        tm.exporting = True
        tm.variable_changed("test", "var1")
        assert tm.prev_values["var1"] == "test"
        assert "var1" not in tm.prev_change
        assert len(tm.prev_values.items()) == 1

    ## Tests if the change_only mode filters receiving the same value reject the change
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testNoPrevChange(self, get_param, ns):
        tm = TriggerMonitor()
        tm.load_config()
        tm.on_change_only = True
        tm.trigger_delay = 0
        
        tm.variable_changed("test", "var1")
        tm.variable_changed("test", "var2")

        t1 = tm.prev_change["var1"]
        t2 = tm.prev_change["var2"]

        tm.variable_changed("test2", "var1")
        tm.variable_changed("test", "var2")

        assert tm.prev_values["var1"] == "test2"
        assert tm.prev_values["var2"] == "test"
        assert tm.prev_change["var2"] == t2
        assert tm.prev_change["var1"] != t1

    ## Tests if the trigger delay works
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testTriggerDelayLess(self, get_param, ns):
        tm = TriggerMonitor()
        tm.load_config()
        tm.on_change_only = True
        tm.trigger_delay = 100
        
        tm.variable_changed("test", "var1")
        t1 = tm.prev_change["var1"]
        tm.variable_changed("test2", "var1")

        assert tm.prev_values["var1"] == "test2"
        assert tm.prev_change["var1"] == t1

    ## Tests if the trigger delay works if the delay is smaller
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testTriggerDelayMore(self, get_param, ns):
        tm = TriggerMonitor()
        tm.load_config()
        tm.on_change_only = True
        tm.trigger_delay = 100
        
        tm.variable_changed("test", "var1")
        t1 = tm.prev_change["var1"]
        time.sleep(0.1)
        tm.variable_changed("test2", "var1")

        assert tm.prev_values["var1"] == "test2"
        assert tm.prev_change["var1"] != t1

    ## Tests if the correct conditions trigger an export
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    @patch('src.trigger_monitor.TriggerMonitor.export')
    def testConditionTrue(self, export, ns, get_param):
        tm = TriggerMonitor()
        tm.load_config()
        tm.conditions["var1"] = "data > 2 and data < 5"
        tm.trigger_delay = 0
        
        for i in range(6):
            tm.variable_changed(i, "var1")

        assert export.call_count == 2
        assert tm.exporting == False
        
    
    ## Tests if the wrong syntax is handled
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    @patch('src.trigger_monitor.TriggerMonitor.formatted_log')
    def testConditionSyntaxError(self, log, ns, get_param):
        tm = TriggerMonitor()
        tm.load_config()
        log.reset_mock()
        tm.conditions["var1"] = "a.b.c"
        
        tm.variable_changed(1, "var1")

        log.assert_called_once()
        log.assert_has_calls([call('w', "<type 'exceptions.NameError'> name 'a' is not defined")])
        assert tm.exporting == False
 
    ## Tests if the export and upload uses the right parameters
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    @patch('src.trigger_monitor.TriggerMonitor.publish_trigger') 
    @patch('src.trigger_monitor.TriggerMonitor.call_upload') 
    @patch('src.trigger_monitor.TriggerMonitor.call_get_available_range', side_effect = get_available_range_response) 
    @patch('src.trigger_monitor.TriggerMonitor.call_export', side_effect = get_export_response) 
    def testExportPublishUploadCalled(self, export, sp, upload, publish, ns, get_param):
        tm = TriggerMonitor() 
        tm.load_config()
          
        tm.export("test", "var1")

        publish.assert_called_once()
        publish.assert_has_calls([call('test', 'var1', 'test', 0, 5)])

        upload.assert_called_once()
        upload.assert_has_calls([call('test', 'test', 'var1')])

    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    @patch('rostopic.get_topic_type', return_value = ['Int32']) 
    @patch('roslib.message.get_message_class') 
    @patch('rospy.Subscriber.__init__', return_value = None) 
    def testTrySubscribeSucceed(self, init_sub, get_class, get_type, ns, get_param):
        tm = TriggerMonitor() 
        tm.load_config()
          
        assert tm.try_subscribe("var1") == True

        init_sub.assert_called_once()
        assert init_sub.call_args[0][0] == 'var1'

    
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    @patch('rostopic.get_topic_type', return_value = [None])
    @patch('roslib.message.get_message_class') 
    @patch('rospy.Subscriber.__init__', return_value = None) 
    def testTrySubscribeFail(self, init_sub, get_class, get_type, ns, get_param):
        tm = TriggerMonitor() 
        tm.load_config()
          
        assert tm.try_subscribe("var1") == False

        init_sub.assert_not_called()

    
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    @patch('rospy.Rate.__init__', return_value = None)
    @patch('src.trigger_monitor.TriggerMonitor.try_subscribe', return_value = False)
    @patch('rospy.is_shutdown', side_effect = is_shutdown)
    @patch('rospy.Rate.sleep', side_effect = rate_sleep)
    def testSubscribe(self, sleep, shutdown, sub, rate, ns, get_param):
        tm = TriggerMonitor() 
        tm.load_config()
        TestTriggerMonitor.is_shutdown = 4

        tm.subscribe('var1')
          
        assert sleep.call_count == 4
        sub.assert_has_calls([call('var1'), call('var1')])

    @patch('src.trigger_monitor.TriggerMonitor.load_config')
    @patch('rospy.spin')
    @patch('rospy.wait_for_service')
    @patch('threading.Thread.__init__', return_value = None)
    @patch('threading.Thread.start')
    @patch('rospy.Publisher.__init__', return_value = None)
    def testStart(self, publisher_init, thread_start, thread_init, wait_for_service, spin, load_config):
        tm = TriggerMonitor() 
        tm.conditions = {}
        tm.conditions["var1"] = ""
        tm.conditions["var2"] = ""
        tm.start()
          
        load_config.assert_called_once()
        wait_for_service.assert_called_once()
        publisher_init.assert_called_once()
        spin.assert_called_once()
        assert thread_init.call_count == 2
        assert thread_start.call_count == 2
        assert thread_init.call_args_list[0][1]['args'][0] == "var1"
        assert thread_init.call_args_list[1][1]['args'][0] == "var2"
