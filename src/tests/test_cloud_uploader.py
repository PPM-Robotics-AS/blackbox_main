from src.cloud_uploader import CloudUploader
import mock
from mock import MagicMock, call, patch
import socket
import ftplib
from blackbox_main.srv import UploadResponse, UploadRequest
from blackbox_main.msg import UploadStatusList, UploadStatus
from rospy import Publisher
from src.ros_node_base import RosNodeBase


class TestCloudUploader(object):
    ## Tests if the formatted log messages are emitted
    @patch('rospy.loginfo')
    @patch('rospy.logwarn')
    @patch('rospy.logerr')
    def testFormattedLog(self, logerr, logwarn, loginfo):
        cu = CloudUploader()
        cu.formatted_log('i', "test")
        loginfo.assert_called_with("[BBU] test")
        cu.formatted_log('w', "test2")
        logwarn.assert_called_with("[BBU] test2")
        cu.formatted_log('e', "test3")
        logerr.assert_called_with("[BBU] test3")
        cu.formatted_log('something else', "test4")
        logerr.assert_called_with("[BBU] test4")

    ## Tests if the member variables are initialized correctly
    def testInit(self):
        cu = CloudUploader()
        assert cu.queue is not None
        assert cu.currentTask is None

    ## Tests if the currect namespace is used
    @patch('rospy.get_name', return_value="xxx")
    def testNs(self, get_name):
        cu = CloudUploader()
        assert cu._ns("test") == "xxx/test"
        assert cu._ns("test2") == "xxx/test2"
        assert cu._ns("test3") == "xxx/test3"

    def patched_get_param(key, def_val):
        return '{0}:{1}'.format(key, def_val)

    def patched_ns(self, key):
        return "/ns/" + key

    ## Tests if the config is loaded correctly
    @patch('rospy.logerr')
    @patch('rospy.logwarn')
    @patch('rospy.get_param', side_effect=patched_get_param)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testLoadConfig(self, ns, get_param, logwarn, logerr):
        cu = CloudUploader()
        cu.load_config()
        #get_param.assert_any_call()
        assert cu.ftp_host == "/ns/ftp_host:None"
        assert cu.ftp_password == "/ns/ftp_password:None"
        assert cu.ftp_username == "/ns/ftp_username:None"
        assert cu.smtp_host == "/ns/smtp_host:None"
        assert cu.smtp_username == "/ns/smtp_username:None"
        assert cu.smtp_password == "/ns/smtp_password:None"
        assert cu.smtp_connection_mode == "/ns/smtp_connection_mode:tls"
        assert cu.smtp_port == "/ns/smtp_port:25"
        assert cu.archive_path == "/ns/archive_path:None"
        assert cu.email_toaddress == "/ns/email_toaddress:None"
        assert cu.email_subject == "/ns/email_subject:New log file uploaded"

        logwarn.assert_not_called()
        logerr.assert_not_called()

    def patched_get_param_missing(key, def_val):
        if key == TestCloudUploader.missing_key:
            return def_val
        else:
            return '{0}:{1}'.format(key, def_val)

    ## Tests if the missing recommended config items are handled correctly
    @patch('sys.exit')
    @patch('rospy.logerr')
    @patch('rospy.logwarn')
    @patch('rospy.get_param', side_effect=patched_get_param_missing)
    @patch('src.ros_node_base.RosNodeBase._ns', side_effect=patched_ns)
    def testLoadConfig_RecommendedMissing(self, ns, get_param, logwarn, logerr, _exit):
        TestCloudUploader.missing_key = "/ns/smtp_host"
        cu = CloudUploader()
        cu.load_config()

        logwarn.assert_called_with('[BBU] Recommended config item not set: smtp_host')
        logerr.assert_not_called()
        _exit.assert_not_called()

        logwarn.reset_mock()
        TestCloudUploader.missing_key = "/ns/ftp_host"
        cu.load_config()

        calls = [call('[BBU] Mandatory config item not set: ftp_host'), call('[BBU] Mandatory config item(s) missing, exiting.')]
        logerr.assert_has_calls(calls)
        logwarn.assert_not_called()
        _exit.assert_called_once()

    ## Tests if the email sending works properly
    @patch('rospy.logwarn')
    @patch('smtplib.SMTP.__init__', return_value = None)
    @patch('smtplib.SMTP.ehlo')
    @patch('smtplib.SMTP.starttls')
    @patch('smtplib.SMTP.login')
    @patch('smtplib.SMTP.sendmail')
    def testSendEmail(self, sendmail, login, starttls, ehlo, smtp, logwarn):
        cu = CloudUploader()
        cu.email_toaddress = "email_to"
        cu.smtp_username = "smtpuser"
        cu.smtp_host = "smtphost"
        cu.smtp_port = "smtpport"
        cu.smtp_password = "smtppass"
        cu.send_mail("bag", "topic", "value", "sub", "body")
        logwarn.assert_not_called()
        assert ehlo.call_count == 2
        starttls.assert_called_once()
        login.assert_called_with("smtpuser","smtppass")
        smtp.assert_called_with("smtphost", "smtpport")

        #test the warning messages, if an exception happens
        del cu.smtp_host
        cu.send_mail("bag", "topic", "value", "sub", "body")
        assert logwarn.call_count == 2

    ## Tests if the upload calls the right underlying functions
    @patch('src.cloud_uploader.CloudUploader.send_mail')
    @patch('ftplib.FTP.quit')
    @patch('ftplib.FTP.storbinary')
    @patch('__builtin__.open')
    @patch('ftplib.FTP.__init__', return_value = None)
    def testUploadAndNotify(self, init, open, stor, quit, send):
        cu = CloudUploader()
        RosNodeBase._recommended_missing = False
        cu.ftp_host = "ftp_host"
        cu.ftp_username = "ftp_username"
        cu.ftp_password = "ftp_password"
        cu.archive_path = "bag_path"
        t = CloudUploader.UploadTask("topic", "value", "filename")
        cu.upload_and_notify(t)
        init.assert_called_with("ftp_host", "ftp_username", "ftp_password")
        open.assert_called_with("bag_path/filename", 'rb')
        quit.assert_called_once()
        send.assert_called_once()
        stor.assert_called_once()

    ## Tests if the upload handles the missing emails data correctly
    @patch('src.cloud_uploader.CloudUploader.send_mail')
    @patch('ftplib.FTP.quit')
    @patch('ftplib.FTP.storbinary')
    @patch('__builtin__.open')
    @patch('ftplib.FTP.__init__', return_value = None)
    def testUploadAndNotifyWithoutEmail(self, init, open, stor, quit, send):
        cu = CloudUploader()
        RosNodeBase._recommended_missing = True
        cu.ftp_host = "ftp_host"
        cu.ftp_username = "ftp_username"
        cu.ftp_password = "ftp_password"
        cu.archive_path = "bag_path"
        t = CloudUploader.UploadTask("topic", "value", "filename")
        cu.upload_and_notify(t)
        send.assert_not_called()
        init.assert_called_with("ftp_host", "ftp_username", "ftp_password")
        open.assert_called_with("bag_path/filename", 'rb')
        quit.assert_called_once()
        stor.assert_called_once()

        

    def ftp_socket_error(host, username, password):
        e = socket.error()
        e.errno = 111
        e.strerror = "Connection refused"
        raise e

    @staticmethod
    def ftp_error_perm(host, username, password):
        e = ftplib.error_perm("530-User cannot log in.")
        raise e

    ## Tests if the FTP errors are handled correctly by the upload function
    @patch('src.cloud_uploader.CloudUploader.formatted_log')
    @patch('src.cloud_uploader.CloudUploader.send_mail')
    @patch('ftplib.FTP.quit')
    @patch('ftplib.FTP.storbinary')
    @patch('__builtin__.open')
    @patch('ftplib.FTP.__init__', return_value = None, side_effect = ftp_socket_error)
    def testUploadAndNotifyFtpErrors(self, init, open, stor, quit, send, log):
        cu = CloudUploader()
        RosNodeBase._recommended_missing = False
        cu.ftp_host = "ftp_host"
        cu.ftp_username = "ftp_username"
        cu.ftp_password = "ftp_password"
        cu.archive_path = "bag_path"
        t = CloudUploader.UploadTask("topic", "value", "filename")
        # Wrong FTP host
        cu.upload_and_notify(t)
        stor.assert_not_called()
        calls = [call('w', 'Cannot upload file: filename'), call('w', "<class 'socket.error'> [Errno 111] Connection refused")]
        log.assert_has_calls(calls)
        send.assert_called_once()

        # Wrong password or user name
        init.side_effect = TestCloudUploader.ftp_error_perm
        send.reset_mock()
        log.reset_mock()
        cu.upload_and_notify(t)
        stor.assert_not_called()
        calls = [call('w', 'Cannot upload file: filename'), call('w', "<class 'ftplib.error_perm'> 530-User cannot log in.")]
        log.assert_has_calls(calls)
        send.assert_called_once()

    ## Tests if the UploadTask is created correctly
    def testUploadTask(self):
        ut = CloudUploader.UploadTask("topic", "value", "filename")
        assert ut.topic == "topic"
        assert ut.value == "value"
        assert ut.filename == "filename"

    def rospy_is_shutdown():
        if TestCloudUploader.rospy_is_shutdown_first_run:
            TestCloudUploader.rospy_is_shutdown_first_run = False
            return False
        else:
            return True

    ## Tests if the upload worker function works correctly
    @patch('rospy.is_shutdown', side_effect = rospy_is_shutdown)
    @patch('src.cloud_uploader.CloudUploader.set_current_task')
    @patch('src.cloud_uploader.CloudUploader.upload_and_notify')
    def testUploadWorker(self, upload, current, shutdown):
        TestCloudUploader.rospy_is_shutdown_first_run = True
        cu = CloudUploader()
        cu.queue.put("test")
        cu.currentTask = "current"
        cu.upload_worker()
        current.assert_has_calls([call(None), call("test")])
        upload.assert_called_with("current")
        assert shutdown.call_count == 2

    ## Tests if the upload callback function works properly
    @patch('src.cloud_uploader.CloudUploader.formatted_log')
    @patch('src.cloud_uploader.CloudUploader.UploadTask.__init__', return_value = None)
    def testUploadCallback(self, init, log):
        cu = CloudUploader()
        data = UploadRequest()
        data.topic = "topic"
        data.value = "value"
        data.filename = "filename"
        resp = cu.upload_callback(data)
        assert log.call_count == 1
        assert log.call_args[0][0] == 'i'
        assert log.call_args[0][1][0:28] == 'New task added to the queue:'
        init.assert_called_with(data.topic, data.value, data.filename)
        assert len(list(cu.queue.queue)) != 0
        assert isinstance(resp, UploadResponse)

    ## Tests if the load callback function works properly
    @patch('src.cloud_uploader.CloudUploader.formatted_log')
    @patch('ftplib.FTP.__init__', return_value = None)
    @patch('ftplib.FTP.quit')
    @patch('ftplib.FTP.retrbinary')
    def testLoadCallbackSuccess(self, retr, quit, init, log):
        cu = CloudUploader()
        data = UploadRequest()
        cu.ftp_host = "ftp_host"
        cu.ftp_username = "ftp_username"
        cu.ftp_password = "ftp_password"
        cu.archive_path = "bag_path"
        data.filename = "filename"
        resp = cu.load_callback(data)
        init.assert_called_with(cu.ftp_host, cu.ftp_username, cu.ftp_password)
        retr.assert_called_once()
        assert retr.call_args[0][0] == 'RETR filename'
        assert isinstance(resp, UploadResponse)

    ## Tests if the load callback function works properly if there is a socket error
    @patch('src.cloud_uploader.CloudUploader.formatted_log')
    @patch('ftplib.FTP.__init__', return_value = None, side_effect = ftp_socket_error)
    @patch('ftplib.FTP.quit')
    @patch('ftplib.FTP.retrbinary')
    def testLoadCallbackSocketError(self, retr, quit, init, log):
        cu = CloudUploader()
        data = UploadRequest()
        cu.ftp_host = "ftp_host"
        cu.ftp_username = "ftp_username"
        cu.ftp_password = "ftp_password"
        cu.archive_path = "bag_path"
        data.filename = "filename"
        resp = cu.load_callback(data)
        init.assert_called_with(cu.ftp_host, cu.ftp_username, cu.ftp_password)
        retr.assert_not_called()
        log.assert_called_with('w', "<class 'socket.error'> [Errno 111] Connection refused")
        assert isinstance(resp, UploadResponse)

    ## Tests if the status publisher worker function works properly
    @patch('rospy.is_shutdown', side_effect = rospy_is_shutdown)
    @patch('rospy.Publisher.publish')
    @patch('time.sleep')
    def testStatusPubWorker(self, sleep, publish, shutdown):
        TestCloudUploader.rospy_is_shutdown_first_run = True
        cu = CloudUploader()
        cu.status_publisher = Publisher("/test", UploadStatusList, queue_size = 0)
        cu.queue.put(CloudUploader.UploadTask("topic", "value", "filename"))
        cu.queue.put(CloudUploader.UploadTask("topic", "value", "filename2"))
        cu.currentTask = CloudUploader.UploadTask("topic", "value", "filename3")
        cu.status_pub_worker()
        assert shutdown.call_count == 2
        publish.assert_called_once()
        assert len(publish.call_args[0][0].statusList) == 3
        assert publish.call_args[0][0].statusList[0].file_name == "filename"
        assert publish.call_args[0][0].statusList[0].uploading == False
        assert publish.call_args[0][0].statusList[1].file_name == "filename2"
        assert publish.call_args[0][0].statusList[1].uploading == False
        assert publish.call_args[0][0].statusList[2].file_name == "filename3"
        assert publish.call_args[0][0].statusList[2].uploading == True
        sleep.assert_called_once()

    ## Tests if the status publisher worker function works properly
    @patch('rospy.is_shutdown', side_effect = rospy_is_shutdown)
    @patch('rospy.Publisher.publish')
    @patch('time.sleep')
    def testStatusPubWorker2(self, sleep, publish, shutdown):
        TestCloudUploader.rospy_is_shutdown_first_run = True
        cu = CloudUploader()
        cu.status_publisher = Publisher("/test", UploadStatusList, queue_size = 0)
        cu.queue.put(CloudUploader.UploadTask("topic", "value", "filename"))
        cu.queue.put(CloudUploader.UploadTask("topic", "value", "filename2"))
        cu.currentTask = None
        cu.status_pub_worker()
        assert shutdown.call_count == 2
        publish.assert_called_once()
        assert len(publish.call_args[0][0].statusList) == 2
        assert publish.call_args[0][0].statusList[0].file_name == "filename"
        assert publish.call_args[0][0].statusList[0].uploading == False
        assert publish.call_args[0][0].statusList[1].file_name == "filename2"
        assert publish.call_args[0][0].statusList[1].uploading == False
        sleep.assert_called_once()

    ## Tests if the start function loads the config and starts the new threads before starting the spin
    @patch('rospy.spin')
    @patch('src.cloud_uploader.CloudUploader.load_config')
    @patch('rospy.Service.__init__', return_value = None)
    @patch('threading.Thread.__init__', return_value = None)
    @patch('threading.Thread.start')
    def testStart(self, thread, thread_start, service, load_config, spin):
        cu = CloudUploader()
        cu.start()        
        load_config.assert_called_once()
        assert service.call_count == 2
        service.call_args[0][0] == "/unnamed/upload"
        assert thread.call_count == 2
        assert thread_start.call_count == 2
        spin.assert_called_once()
