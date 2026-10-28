#include "c3d/remote_variables.hpp"

#include "c3d/session.hpp"

#include <nlohmann/json.hpp>

#include "json_dump.hpp"
#include "c3d/sha256.hpp"

namespace c3d {
namespace {

using Json = nlohmann::ordered_json;

// One item. Returns false with `why` filled for anything the platform should not have sent.
bool ReadItem(const Json& item, RemoteVariable* out, std::string* why) {
  const auto name_it = item.find("remoteVariableName");
  if (name_it == item.end() || !name_it->is_string() || name_it->get<std::string>().empty()) {
    *why = "an item has no remoteVariableName";
    return false;
  }
  const std::string name = name_it->get<std::string>();
  const auto type_it = item.find("type");
  if (type_it == item.end() || !type_it->is_string()) {
    *why = "'" + name + "' has no string type";
    return false;
  }
  const std::string type = type_it->get<std::string>();

  const char* field = nullptr;
  if (type == "int") field = "valueInt";
  else if (type == "boolean") field = "valueBoolean";
  else if (type == "string") field = "valueString";
  if (field == nullptr) {
    *why = "'" + name + "' has type '" + type + "', which is not int, boolean or string";
    return false;
  }

  const auto value_it = item.find(field);
  if (value_it == item.end() || value_it->is_null()) {
    *why = "'" + name + "' is typed " + type + " but carries no " + field;
    return false;
  }
  if (type == "int") {
    if (!value_it->is_number_integer()) {
      *why = "'" + name + "' has a " + field + " that is not an integer";
      return false;
    }
    out->value = PropertyValue(value_it->get<std::int64_t>());
  } else if (type == "boolean") {
    if (!value_it->is_boolean()) {
      *why = "'" + name + "' has a " + field + " that is not a boolean";
      return false;
    }
    out->value = PropertyValue(value_it->get<bool>());
  } else {
    if (!value_it->is_string()) {
      *why = "'" + name + "' has a " + field + " that is not a string";
      return false;
    }
    out->value = PropertyValue(value_it->get<std::string>());
  }
  out->name = name;
  return true;
}

}  // namespace

std::string RemoteVariablesUrl(const Config& config, const std::string& identifier) {
  return config.protocol + "://" + config.gateway + "/v" + config.network_version +
         "/remotevariables?identifier=" + Sha256Hex(identifier);
}

bool ParseRemoteVariables(const std::string& body, RemoteVariableSet* out, std::string* error) {
  if (out == nullptr) return false;
  Json doc;
  try {
    doc = Json::parse(body);
  } catch (const std::exception& e) {
    if (error) *error = std::string("could not parse remote variables JSON: ") + e.what();
    return false;
  }
  if (!doc.is_object()) {
    if (error) {
      *error = "remote variables JSON must be an object, got " + std::string(doc.type_name());
    }
    return false;
  }

  std::vector<RemoteVariable> variables;
  std::vector<std::string> skipped;
  for (const char* key : {"abTests", "remoteConfigurations"}) {
    const auto array_it = doc.find(key);
    // Absent and null both mean "no items of this kind". Anything else present under the key is
    // a response the platform should not have sent, and silently reading it as empty would turn
    // a whole array of arms into a session in no arm with nothing to say why.
    if (array_it == doc.end() || array_it->is_null()) continue;
    if (!array_it->is_array()) {
      if (error) *error = std::string(key) + " is present but is not an array";
      return false;
    }
    for (const auto& item : *array_it) {
      RemoteVariable variable;
      std::string why;
      if (item.is_object() && ReadItem(item, &variable, &why)) {
        variables.push_back(std::move(variable));
      } else {
        skipped.push_back(std::string(key) + ": " + (why.empty() ? "an item is not an object" : why));
      }
    }
  }
  out->variables = std::move(variables);
  out->skipped = std::move(skipped);
  return true;
}

RemoteVariableSet FetchRemoteVariables(IHttpGet& reader, const Config& config,
                                       const std::string& identifier) {
  RemoteVariableSet set;
  const std::vector<std::string> headers = {
      "Content-Type: application/json",
      "Authorization: APIKEY:DATA " + config.application_key,
      // Unity sends this on every read of this route. Harmless if the gateway ignores it,
      // and it keeps the request byte-identical to the reference implementation's.
      "X-HTTP-Method-Override: GET",
  };
  const HttpResponse response = reader.Get(RemoteVariablesUrl(config, identifier), headers);
  set.status = response.status;

  if (!response.IsSuccess()) {
    if (response.status == 200) {
      set.error = "200 without the cvr-request-time header: not the Cognitive3D gateway";
    } else if (response.status == 0) {
      set.error = response.error.empty() ? "no response" : response.error;
    } else {
      set.error = "HTTP " + std::to_string(response.status);
    }
    return set;
  }

  std::string error;
  if (!ParseRemoteVariables(response.body, &set, &error)) {
    set.variables.clear();
    set.error = error;
    return set;
  }
  set.fetched = true;
  return set;
}

std::string RemoteVariableSet::ToJson() const {
  Json object = Json::object();
  for (const auto& variable : variables) {
    switch (variable.value.kind()) {
      case PropertyValue::Kind::kInt: object[variable.name] = variable.value.as_int(); break;
      case PropertyValue::Kind::kBool: object[variable.name] = variable.value.as_bool(); break;
      case PropertyValue::Kind::kDouble: object[variable.name] = variable.value.as_double(); break;
      case PropertyValue::Kind::kString: object[variable.name] = variable.value.as_string(); break;
    }
  }
  return DumpJson(object);
}

void ApplyRemoteVariables(Session& session, const RemoteVariableSet& set) {
  for (const auto& variable : set.variables) {
    session.SetProperty("c3d.remote_variable." + variable.name, variable.value);
  }
}

}  // namespace c3d
