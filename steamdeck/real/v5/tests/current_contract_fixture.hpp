#pragma once
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include "../../../../rt-core/clients/cpp/include/rosie/rt_control_api_generated.hpp"

// Reuse recorded poses/topology with the candidate's protocol and OLP features.
// Compatibility rejection itself is covered by targets/rtCore.test and machine.test.
// This only changes the synthetic server response; original captures stay intact.
inline QJsonValue currentContractFixture(QJsonValue value) {
 if(value.isArray()) { auto a=value.toArray(); for(int i=0;i<a.size();++i) a[i]=currentContractFixture(a[i]); return a; }
 if(!value.isObject()) return value;
 auto object=value.toObject();
 for(auto it=object.begin();it!=object.end();++it) it.value()=currentContractFixture(it.value());
 if(object.value("backend")=="rt_core" && object.contains("daemon_reachable")) object["recover_zero_jog_speeds"]=true;
 if(object.contains("capabilities_digest")) object["capabilities_digest"]=rosie::rt_control::CapabilitiesDigest;
 return object;
}
inline QByteArray currentContractFixture(const QByteArray& bytes) {
 QJsonParseError error; const auto doc=QJsonDocument::fromJson(bytes,&error);
 if(error.error!=QJsonParseError::NoError) return bytes;
 return QJsonDocument(currentContractFixture(doc.object()).toObject()).toJson(QJsonDocument::Compact);
}
