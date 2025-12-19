// RosPlugin.js
// Bu kod, ROS'tan gelen veriyi OpenMCT formatına çevirir.

const ROSLIB = require('roslib'); // Veya script tag kullandiysan globalden gelir

function RosPlugin() {
    return function install(openmct) {
        
        // 1. ROS Bridge'e Bağlan (9090 Portu)
        var ros = new ROSLIB.Ros({
            url : 'ws://localhost:9090'
        });

        ros.on('connection', function() {
            console.log('OpenMCT: ROS 2 Bridge sunucusuna bağlandı!');
        });

        ros.on('error', function(error) {
            console.log('OpenMCT: ROS bağlantı hatası:', error);
        });

        // 2. Telemetry Sağlayıcısını Tanımla
        var provider = {
            // Sadece 'example.ros' tipindeki objeler için çalışır
            supportsSubscribe: function (domainObject) {
                return domainObject.type === 'example.ros';
            },
            
            // Abone olma işlemi (Subscribe)
            subscribe: function (domainObject, callback) {
                // Objeden topic adını al (örneğin: /battery_level)
                var topicName = domainObject.telemetry.topic;
                
                var listener = new ROSLIB.Topic({
                    ros : ros,
                    name : topicName,
                    messageType : 'std_msgs/Float32' // Kendi mesaj tipine göre değiştir!
                });

                listener.subscribe(function(message) {
                    // ROS verisini OpenMCT formatına çevir
                    // OpenMCT genelde { timestamp, value, id } bekler
                    var datum = {
                        timestamp: Date.now(),
                        value: message.data, 
                        id: domainObject.identifier.key
                    };
                    callback(datum);
                });

                // Aboneliği iptal etme fonksiyonunu döndür
                return function unsubscribe() {
                    listener.unsubscribe();
                };
            }
        };

        // Sağlayıcıyı OpenMCT'ye kaydet
        openmct.telemetry.addProvider(provider);
    };
}

module.exports = RosPlugin;
