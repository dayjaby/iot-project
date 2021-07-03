
// Copyright 2012 Google Inc. All rights reserved.
(function(){

var data = {
"resource": {
  "version":"50",
  
  "macros":[{
      "function":"__v",
      "vtp_name":"gtm.elementUrl",
      "vtp_dataLayerVersion":1
    },{
      "function":"__e"
    },{
      "function":"__v",
      "vtp_name":"gtm.triggers",
      "vtp_dataLayerVersion":2,
      "vtp_setDefaultValue":true,
      "vtp_defaultValue":""
    },{
      "function":"__u",
      "vtp_component":"URL",
      "vtp_enableMultiQueryKeys":false,
      "vtp_enableIgnoreEmptyQueryParam":false
    },{
      "function":"__u",
      "vtp_component":"HOST",
      "vtp_enableMultiQueryKeys":false,
      "vtp_enableIgnoreEmptyQueryParam":false
    },{
      "function":"__smm",
      "vtp_setDefaultValue":true,
      "vtp_input":["macro",4],
      "vtp_defaultValue":"UA-3800502-33",
      "vtp_map":["list",["map","key","localhost","value","UA-3800502-33"],["map","key","mbed-developer-qa.test.mbed.com","value","UA-3800502-33"],["map","key","os.mbed.com","value","UA-1447836-8"],["map","key","account.mbed.com","value","UA-1447836-8"]]
    },{
      "function":"__gas",
      "vtp_cookieDomain":"auto",
      "vtp_doubleClick":false,
      "vtp_setTrackerName":false,
      "vtp_useDebugVersion":false,
      "vtp_useHashAutoLink":false,
      "vtp_decorateFormsAutoLink":false,
      "vtp_enableLinkId":false,
      "vtp_enableEcommerce":false,
      "vtp_trackingId":"UA-1447836-8",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableGA4Schema":false
    },{
      "function":"__j",
      "vtp_name":"navigator.userAgent"
    },{
      "function":"__v",
      "vtp_name":"gtm.element",
      "vtp_dataLayerVersion":1
    },{
      "function":"__v",
      "vtp_name":"gtm.elementId",
      "vtp_dataLayerVersion":1
    },{
      "function":"__u",
      "vtp_component":"PATH",
      "vtp_enableMultiQueryKeys":false,
      "vtp_enableIgnoreEmptyQueryParam":false
    },{
      "function":"__gas",
      "vtp_cookieDomain":"auto",
      "vtp_doubleClick":false,
      "vtp_setTrackerName":false,
      "vtp_useDebugVersion":false,
      "vtp_useHashAutoLink":false,
      "vtp_decorateFormsAutoLink":false,
      "vtp_enableLinkId":false,
      "vtp_enableEcommerce":false,
      "vtp_trackingId":"UA-3800502-33",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableGA4Schema":false
    },{
      "function":"__d",
      "vtp_elementId":"activate-button",
      "vtp_selectorType":"ID"
    },{
      "function":"__d",
      "vtp_elementSelector":".button",
      "vtp_selectorType":"CSS"
    },{
      "function":"__gas",
      "vtp_cookieDomain":"auto",
      "vtp_doubleClick":false,
      "vtp_setTrackerName":false,
      "vtp_useDebugVersion":false,
      "vtp_useHashAutoLink":false,
      "vtp_decorateFormsAutoLink":false,
      "vtp_enableLinkId":false,
      "vtp_enableEcommerce":false,
      "vtp_trackingId":["macro",5],
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableGA4Schema":false
    },{
      "function":"__f",
      "vtp_component":"URL"
    },{
      "function":"__e"
    },{
      "function":"__v",
      "vtp_name":"gtm.elementClasses",
      "vtp_dataLayerVersion":1
    },{
      "function":"__v",
      "vtp_name":"gtm.elementId",
      "vtp_dataLayerVersion":1
    },{
      "function":"__v",
      "vtp_name":"gtm.elementTarget",
      "vtp_dataLayerVersion":1
    },{
      "function":"__aev",
      "vtp_varType":"TEXT"
    },{
      "function":"__v",
      "vtp_name":"gtm.element",
      "vtp_dataLayerVersion":1
    },{
      "function":"__v",
      "vtp_name":"gtm.elementClasses",
      "vtp_dataLayerVersion":1
    }],
  "tags":[{
      "function":"__ua",
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":true,
      "vtp_eventCategory":"outbound",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_eventAction":["macro",0],
      "vtp_eventLabel":["macro",3],
      "vtp_trackingId":["macro",5],
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":1
    },{
      "function":"__ua",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_overrideGaSettings":true,
      "vtp_trackType":"TRACK_PAGEVIEW",
      "vtp_gaSettings":["macro",6],
      "vtp_dimension":["list",["map","index","1","dimension",["macro",7]]],
      "vtp_trackingId":["macro",5],
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_enableGA4Schema":false,
      "tag_id":2
    },{
      "function":"__ua",
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":true,
      "vtp_eventCategory":"register",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_eventAction":["macro",0],
      "vtp_eventLabel":["macro",3],
      "vtp_trackingId":["macro",5],
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":3
    },{
      "function":"__ua",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":true,
      "vtp_eventCategory":"download",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_eventAction":["macro",0],
      "vtp_eventLabel":["macro",3],
      "vtp_trackingId":["macro",5],
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":4
    },{
      "function":"__ua",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":false,
      "vtp_eventCategory":"Button clicks",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_gaSettings":["macro",6],
      "vtp_eventAction":"Click",
      "vtp_eventLabel":"Activate Pelion free tier",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":8
    },{
      "function":"__hjtc",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_hotjar_site_id":"971746",
      "tag_id":10
    },{
      "function":"__opt",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_overrideGaSettings":true,
      "vtp_setTrackerName":false,
      "vtp_useDebugVersion":false,
      "vtp_optimizeContainerId":"GTM-WXMLD4S",
      "vtp_trackingId":["macro",5],
      "tag_id":11
    },{
      "function":"__ua",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":false,
      "vtp_eventCategory":"Login",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_gaSettings":["macro",6],
      "vtp_eventAction":"Submit",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":14
    },{
      "function":"__ua",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":false,
      "vtp_eventCategory":"Register",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_gaSettings":["macro",6],
      "vtp_eventAction":"Create account",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":15
    },{
      "function":"__ua",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":false,
      "vtp_eventCategory":"Account actions",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_gaSettings":["macro",6],
      "vtp_eventAction":"Add notebook",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":67
    },{
      "function":"__ua",
      "metadata":["map"],
      "consent":["list"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":false,
      "vtp_eventCategory":"Account actions",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_gaSettings":["macro",6],
      "vtp_eventAction":"Edit notebook",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":69
    },{
      "function":"__ua",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":false,
      "vtp_eventCategory":"Register",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_gaSettings":["macro",6],
      "vtp_eventAction":"Landing page CTA",
      "vtp_eventLabel":"Keil Studio Preview",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":71
    },{
      "function":"__ua",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":false,
      "vtp_eventCategory":"Account actions",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_gaSettings":["macro",6],
      "vtp_eventAction":"Create hg repo",
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":73
    },{
      "function":"__ua",
      "metadata":["map"],
      "consent":["list"],
      "once_per_event":true,
      "vtp_nonInteraction":false,
      "vtp_overrideGaSettings":true,
      "vtp_eventCategory":"download",
      "vtp_trackType":"TRACK_EVENT",
      "vtp_gaSettings":["macro",6],
      "vtp_eventAction":["macro",0],
      "vtp_eventLabel":["macro",3],
      "vtp_enableRecaptchaOption":false,
      "vtp_enableUaRlsa":false,
      "vtp_enableUseInternalVersion":false,
      "vtp_enableFirebaseCampaignData":true,
      "vtp_trackTypeIsEvent":true,
      "vtp_enableGA4Schema":false,
      "tag_id":81
    },{
      "function":"__lcl",
      "vtp_waitForTags":true,
      "vtp_checkValidation":true,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_5",
      "tag_id":82
    },{
      "function":"__lcl",
      "vtp_waitForTags":true,
      "vtp_checkValidation":true,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_33",
      "tag_id":83
    },{
      "function":"__lcl",
      "vtp_waitForTags":true,
      "vtp_checkValidation":true,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_34",
      "tag_id":84
    },{
      "function":"__lcl",
      "vtp_waitForTags":true,
      "vtp_checkValidation":false,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_38",
      "tag_id":85
    },{
      "function":"__lcl",
      "vtp_waitForTags":true,
      "vtp_checkValidation":false,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_39",
      "tag_id":86
    },{
      "function":"__lcl",
      "vtp_waitForTags":true,
      "vtp_checkValidation":false,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_40",
      "tag_id":87
    },{
      "function":"__cl",
      "tag_id":88
    },{
      "function":"__evl",
      "vtp_elementId":"signup-form",
      "vtp_useOnScreenDuration":false,
      "vtp_useDomChangeListener":true,
      "vtp_firingFrequency":"ONCE",
      "vtp_selectorType":"ID",
      "vtp_onScreenRatio":"1",
      "vtp_uniqueTriggerId":"8441108_55",
      "tag_id":89
    },{
      "function":"__fsl",
      "vtp_checkValidation":false,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_59",
      "tag_id":90
    },{
      "function":"__fsl",
      "vtp_checkValidation":false,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_61",
      "tag_id":91
    },{
      "function":"__lcl",
      "vtp_waitForTags":true,
      "vtp_checkValidation":false,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_64",
      "tag_id":92
    },{
      "function":"__fsl",
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_65",
      "tag_id":93
    },{
      "function":"__fsl",
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_68",
      "tag_id":94
    },{
      "function":"__lcl",
      "vtp_waitForTags":false,
      "vtp_checkValidation":false,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_70",
      "tag_id":95
    },{
      "function":"__fsl",
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_72",
      "tag_id":96
    },{
      "function":"__lcl",
      "vtp_waitForTags":false,
      "vtp_checkValidation":false,
      "vtp_waitForTagsTimeout":"2000",
      "vtp_uniqueTriggerId":"8441108_80",
      "tag_id":97
    },{
      "function":"__html",
      "metadata":["map"],
      "once_per_event":true,
      "vtp_html":"\u003Cscript data-gtmsrc=\"https:\/\/1eb17132b4904a06a7e688e4ebf4ccf3.js.ubembed.com\" async type=\"text\/gtmscript\"\u003E\u003C\/script\u003E",
      "vtp_supportDocumentWrite":false,
      "vtp_enableIframeMode":false,
      "vtp_enableEditJsMacroBehavior":false,
      "tag_id":13
    }],
  "predicates":[{
      "function":"_cn",
      "arg0":["macro",0],
      "arg1":"mbed.com"
    },{
      "function":"_cn",
      "arg0":["macro",0],
      "arg1":"localhost"
    },{
      "function":"_cn",
      "arg0":["macro",0],
      "arg1":"mbed.org"
    },{
      "function":"_eq",
      "arg0":["macro",1],
      "arg1":"gtm.linkClick"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_5($|,)))"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_33($|,)))"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_34($|,)))"
    },{
      "function":"_eq",
      "arg0":["macro",1],
      "arg1":"gtm.js"
    },{
      "function":"_cn",
      "arg0":["macro",0],
      "arg1":"\/studio\/register"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_38($|,)))"
    },{
      "function":"_ew",
      "arg0":["macro",0],
      "arg1":".exe"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_39($|,)))"
    },{
      "function":"_ew",
      "arg0":["macro",0],
      "arg1":".pkg"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_40($|,)))"
    },{
      "function":"_ew",
      "arg0":["macro",0],
      "arg1":".sh"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_64($|,)))"
    },{
      "function":"_css",
      "arg0":["macro",8],
      "arg1":"#activate-button"
    },{
      "function":"_cn",
      "arg0":["macro",4],
      "arg1":"test"
    },{
      "function":"_cn",
      "arg0":["macro",4],
      "arg1":"localhost"
    },{
      "function":"_eq",
      "arg0":["macro",1],
      "arg1":"gtm.click"
    },{
      "function":"_cn",
      "arg0":["macro",4],
      "arg1":"account"
    },{
      "function":"_eq",
      "arg0":["macro",9],
      "arg1":"login-form"
    },{
      "function":"_eq",
      "arg0":["macro",1],
      "arg1":"gtm.formSubmit"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_59($|,)))"
    },{
      "function":"_eq",
      "arg0":["macro",9],
      "arg1":"signup-form"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_61($|,)))"
    },{
      "function":"_ew",
      "arg0":["macro",10],
      "arg1":"\/notebook"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_65($|,)))"
    },{
      "function":"_re",
      "arg0":["macro",10],
      "arg1":"\\\/users\\\/.*\\\/notebook\\\/.*\\\/edit"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_68($|,)))"
    },{
      "function":"_eq",
      "arg0":["macro",0],
      "arg1":"https:\/\/surveys.hotjar.com\/871d3656-0367-4281-a622-e3f52371d9da"
    },{
      "function":"_eq",
      "arg0":["macro",3],
      "arg1":"https:\/\/os.mbed.com\/keil\/"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_70($|,)))"
    },{
      "function":"_eq",
      "arg0":["macro",0],
      "arg1":"https:\/\/os.mbed.com\/code\/create"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_72($|,)))"
    },{
      "function":"_ew",
      "arg0":["macro",0],
      "arg1":".pdf"
    },{
      "function":"_cn",
      "arg0":["macro",10],
      "arg1":"\/Results-of-2020-Mbed-Developer-Survey\/"
    },{
      "function":"_re",
      "arg0":["macro",2],
      "arg1":"(^$|((^|,)8441108_80($|,)))"
    },{
      "function":"_cn",
      "arg0":["macro",10],
      "arg1":"modules"
    },{
      "function":"_cn",
      "arg0":["macro",10],
      "arg1":"platforms"
    },{
      "function":"_cn",
      "arg0":["macro",10],
      "arg1":"components"
    },{
      "function":"_cn",
      "arg0":["macro",10],
      "arg1":"\/studio"
    }],
  "rules":[
    [["if",3,4],["unless",0,1,2],["add",0]],
    [["if",3,5],["unless",0,1,2],["add",0]],
    [["if",3,6],["unless",0,1,2],["add",0]],
    [["if",7],["add",1,30,20,21,22,23,25,26,27,28,29]],
    [["if",3,8,9],["add",2]],
    [["if",3,10,11],["add",3]],
    [["if",3,12,13],["add",3]],
    [["if",3,14,15],["add",3]],
    [["if",16,19],["unless",17,18],["add",4]],
    [["if",7,20],["add",5,6]],
    [["if",21,22,23],["add",7]],
    [["if",22,24,25],["add",8]],
    [["if",22,26,27],["add",9]],
    [["if",22,28,29],["add",10]],
    [["if",3,30,31,32],["add",11]],
    [["if",22,33,34],["add",12]],
    [["if",3,35,36,37],["add",13]],
    [["if",7,38],["add",14]],
    [["if",7,39],["add",15]],
    [["if",7,40],["add",16]],
    [["if",7,41],["add",17,18,19,24]]]
},
"runtime":[[50,"__hjtc",[46,"a"],[52,"b",["require","createArgumentsQueue"]],[52,"c",["require","encodeUriComponent"]],[52,"d",["require","injectScript"]],[52,"e",["require","makeString"]],[52,"f",["require","setInWindow"]],["b","hj","hj.q"],[52,"g",[17,[15,"a"],"hotjar_site_id"]],["f","_hjSettings",[8,"hjid",[15,"g"],"hjsv",7,"scriptSource","gtm"]],["d",[0,[0,"https://static.hotjar.com/c/hotjar-",["c",["e",[15,"g"]]]],".js?sv\u003d7"],[17,[15,"a"],"gtmOnSuccess"],[17,[15,"a"],"gtmOnFailure"]]]]
,"permissions":{"__hjtc":{"access_globals":{"keys":[{"key":"hj","read":true,"write":true,"execute":false},{"key":"hj.q","read":true,"write":true,"execute":false},{"key":"_hjSettings","read":true,"write":true,"execute":false}]},"inject_script":{"urls":["https:\/\/static.hotjar.com\/c\/hotjar-*"]}}}

,"security_groups":{
"nonGoogleScripts":["__hjtc"]}

};

/*

 Copyright The Closure Library Authors.
 SPDX-License-Identifier: Apache-2.0
*/
var aa,ca=function(a){var b=0;return function(){return b<a.length?{done:!1,value:a[b++]}:{done:!0}}},da=function(a){var b="undefined"!=typeof Symbol&&Symbol.iterator&&a[Symbol.iterator];return b?b.call(a):{next:ca(a)}},ea="function"==typeof Object.create?Object.create:function(a){var b=function(){};b.prototype=a;return new b},fa;
if("function"==typeof Object.setPrototypeOf)fa=Object.setPrototypeOf;else{var ha;a:{var ja={a:!0},la={};try{la.__proto__=ja;ha=la.a;break a}catch(a){}ha=!1}fa=ha?function(a,b){a.__proto__=b;if(a.__proto__!==b)throw new TypeError(a+" is not extensible");return a}:null}
var ma=fa,oa=function(a,b){a.prototype=ea(b.prototype);a.prototype.constructor=a;if(ma)ma(a,b);else for(var c in b)if("prototype"!=c)if(Object.defineProperties){var d=Object.getOwnPropertyDescriptor(b,c);d&&Object.defineProperty(a,c,d)}else a[c]=b[c];a.pj=b.prototype},pa=this||self,qa=function(a){return a};var ra=function(a,b){this.g=a;this.s=b};var sa=function(a){return"number"===typeof a&&0<=a&&isFinite(a)&&0===a%1||"string"===typeof a&&"-"!==a[0]&&a===""+parseInt(a,10)},ua=function(){this.B={};this.F=!1;this.L={}},va=function(a,b){var c=[],d;for(d in a.B)if(a.B.hasOwnProperty(d))switch(d=d.substr(5),b){case 1:c.push(d);break;case 2:c.push(a.get(d));break;case 3:c.push([d,a.get(d)])}return c};ua.prototype.get=function(a){return this.B["dust."+a]};ua.prototype.set=function(a,b){this.F||(a="dust."+a,this.L.hasOwnProperty(a)||(this.B[a]=b))};
ua.prototype.has=function(a){return this.B.hasOwnProperty("dust."+a)};var wa=function(a,b){b="dust."+b;a.F||a.L.hasOwnProperty(b)||delete a.B[b]};ua.prototype.mb=function(){this.F=!0};var k=function(a){this.s=new ua;this.g=[];this.B=!1;a=a||[];for(var b in a)a.hasOwnProperty(b)&&(sa(b)?this.g[Number(b)]=a[Number(b)]:this.s.set(b,a[b]))};aa=k.prototype;aa.toString=function(a){if(a&&0<=a.indexOf(this))return"";for(var b=[],c=0;c<this.g.length;c++){var d=this.g[c];null===d||void 0===d?b.push(""):d instanceof k?(a=a||[],a.push(this),b.push(d.toString(a)),a.pop()):b.push(d.toString())}return b.join(",")};
aa.set=function(a,b){if(!this.B)if("length"===a){if(!sa(b))throw Error("RangeError: Length property must be a valid integer.");this.g.length=Number(b)}else sa(a)?this.g[Number(a)]=b:this.s.set(a,b)};aa.get=function(a){return"length"===a?this.length():sa(a)?this.g[Number(a)]:this.s.get(a)};aa.length=function(){return this.g.length};aa.lb=function(){for(var a=va(this.s,1),b=0;b<this.g.length;b++)a.push(b+"");return new k(a)};var xa=function(a,b){sa(b)?delete a.g[Number(b)]:wa(a.s,b)};aa=k.prototype;
aa.pop=function(){return this.g.pop()};aa.push=function(a){return this.g.push.apply(this.g,Array.prototype.slice.call(arguments))};aa.shift=function(){return this.g.shift()};aa.splice=function(a,b,c){return new k(this.g.splice.apply(this.g,arguments))};aa.unshift=function(a){return this.g.unshift.apply(this.g,Array.prototype.slice.call(arguments))};aa.has=function(a){return sa(a)&&this.g.hasOwnProperty(a)||this.s.has(a)};aa.mb=function(){this.B=!0;Object.freeze(this.g);this.s.mb()};var za=function(){function a(f,g){if(b[f]){if(b[f].gd+g>b[f].max)throw Error("Quota exceeded");b[f].gd+=g}}var b={},c=void 0,d=void 0,e={Ci:function(f){c=f},fg:function(){c&&a(c,1)},Ei:function(f){d=f},nb:function(f){d&&a(d,f)},Ti:function(f,g){b[f]=b[f]||{gd:0};b[f].max=g},di:function(f){return b[f]&&b[f].gd||0},reset:function(){b={}},Sh:a};e.onFnConsume=e.Ci;e.consumeFn=e.fg;e.onStorageConsume=e.Ei;e.consumeStorage=e.nb;e.setMax=e.Ti;e.getConsumed=e.di;e.reset=e.reset;e.consume=e.Sh;return e};var Aa=function(a,b){this.B=a;this.P=function(c,d,e){return c.apply(d,e)};this.F=b;this.s=new ua;this.g=this.L=void 0};Aa.prototype.add=function(a,b){Ba(this,a,b,!1)};var Ba=function(a,b,c,d){if(!a.s.F)if(a.B.nb(("string"===typeof b?b.length:1)+("string"===typeof c?c.length:1)),d){var e=a.s;e.set(b,c);e.L["dust."+b]=!0}else a.s.set(b,c)};
Aa.prototype.set=function(a,b){this.s.F||(!this.s.has(a)&&this.F&&this.F.has(a)?this.F.set(a,b):(this.B.nb(("string"===typeof a?a.length:1)+("string"===typeof b?b.length:1)),this.s.set(a,b)))};Aa.prototype.get=function(a){return this.s.has(a)?this.s.get(a):this.F?this.F.get(a):void 0};Aa.prototype.has=function(a){return!!this.s.has(a)||!(!this.F||!this.F.has(a))};var Da=function(a){var b=new Aa(a.B,a);a.L&&(b.L=a.L);b.P=a.P;b.g=a.g;return b};var Ea={},Fa=function(a,b){Ea[a]=Ea[a]||[];Ea[a][b]=!0},Ha=function(a){for(var b=[],c=Ea[a]||[],d=0;d<c.length;d++)c[d]&&(b[Math.floor(d/6)]^=1<<d%6);for(var e=0;e<b.length;e++)b[e]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_".charAt(b[e]||0);return b.join("")};var Ia=function(){},Ja=function(a){return"function"==typeof a},n=function(a){return"string"==typeof a},La=function(a){return"number"==typeof a&&!isNaN(a)},Na=function(a){var b="[object Array]"==Object.prototype.toString.call(Object(a));Array.isArray?Array.isArray(a)!==b&&Fa("TAGGING",4):Fa("TAGGING",5);return b},Oa=function(a,b){if(Array.prototype.indexOf){var c=a.indexOf(b);return"number"==typeof c?c:-1}for(var d=0;d<a.length;d++)if(a[d]===b)return d;return-1},Qa=function(a,b){if(a&&Na(a))for(var c=
0;c<a.length;c++)if(a[c]&&b(a[c]))return a[c]},Ra=function(a,b){if(!La(a)||!La(b)||a>b)a=0,b=2147483647;return Math.floor(Math.random()*(b-a+1)+a)},Ta=function(a,b){for(var c=new Sa,d=0;d<a.length;d++)c.set(a[d],!0);for(var e=0;e<b.length;e++)if(c.get(b[e]))return!0;return!1},Ua=function(a,b){for(var c in a)Object.prototype.hasOwnProperty.call(a,c)&&b(c,a[c])},Va=function(a){return!!a&&("[object Arguments]"==Object.prototype.toString.call(a)||Object.prototype.hasOwnProperty.call(a,"callee"))},Wa=
function(a){return Math.round(Number(a))||0},Xa=function(a){return"false"==String(a).toLowerCase()?!1:!!a},Ya=function(a){var b=[];if(Na(a))for(var c=0;c<a.length;c++)b.push(String(a[c]));return b},Za=function(a){return a?a.replace(/^\s+|\s+$/g,""):""},$a=function(){return new Date(Date.now())},ab=function(){return $a().getTime()},Sa=function(){this.prefix="gtm.";this.values={}};Sa.prototype.set=function(a,b){this.values[this.prefix+a]=b};
Sa.prototype.get=function(a){return this.values[this.prefix+a]};
var bb=function(a,b,c){return a&&a.hasOwnProperty(b)?a[b]:c},eb=function(a){var b=a;return function(){if(b){var c=b;b=void 0;try{c()}catch(d){}}}},gb=function(a,b){for(var c in b)b.hasOwnProperty(c)&&(a[c]=b[c])},hb=function(a){for(var b in a)if(a.hasOwnProperty(b))return!0;return!1},ib=function(a,b){for(var c=[],d=0;d<a.length;d++)c.push(a[d]),c.push.apply(c,b[a[d]]||[]);return c},jb=function(a,b){var c=A;b=b||[];for(var d=c,e=0;e<a.length-1;e++){if(!d.hasOwnProperty(a[e]))return;d=d[a[e]];if(0<=
Oa(b,d))return}return d},kb=function(a,b){for(var c={},d=c,e=a.split("."),f=0;f<e.length-1;f++)d=d[e[f]]={};d[e[e.length-1]]=b;return c},lb=/^\w{1,9}$/,mb=function(a){var b=[];Ua(a,function(c,d){lb.test(c)&&d&&b.push(c)});return b.join(",")};var pb=function(a,b){ua.call(this);this.P=a;this.Ha=b};oa(pb,ua);pb.prototype.toString=function(){return this.P};pb.prototype.lb=function(){return new k(va(this,1))};pb.prototype.g=function(a,b){a.B.fg();return this.Ha.apply(new qb(this,a),Array.prototype.slice.call(arguments,1))};pb.prototype.s=function(a,b){try{return this.g.apply(this,Array.prototype.slice.call(arguments,0))}catch(c){}};
var sb=function(a,b){for(var c,d=0;d<b.length&&!(c=rb(a,b[d]),c instanceof ra);d++);return c},rb=function(a,b){try{var c=a.get(String(b[0]));if(!(c&&c instanceof pb))throw Error("Attempting to execute non-function "+b[0]+".");return c.g.apply(c,[a].concat(b.slice(1)))}catch(e){var d=a.L;d&&d(e,b.context?{id:b[0],line:b.context.line}:null);throw e;}},qb=function(a,b){this.s=a;this.g=b},F=function(a,b){var c=a.g;return Na(b)?rb(c,b):b},G=function(a){return a.s.P};var tb=function(){ua.call(this)};oa(tb,ua);tb.prototype.lb=function(){return new k(va(this,1))};var vb={control:function(a,b){return new ra(a,F(this,b))},fn:function(a,b,c){var d=this.g,e=F(this,b);if(!(e instanceof k))throw Error("Error: non-List value given for Fn argument names.");var f=Array.prototype.slice.call(arguments,2);this.g.B.nb(a.length+f.length);return new pb(a,function(){return function(g){var h=Da(d);void 0===h.g&&(h.g=this.g.g);for(var l=Array.prototype.slice.call(arguments,0),m=0;m<l.length;m++)if(l[m]=F(this,l[m]),l[m]instanceof ra)return l[m];for(var p=e.get("length"),q=
0;q<p;q++)q<l.length?h.add(e.get(q),l[q]):h.add(e.get(q),void 0);h.add("arguments",new k(l));var r=sb(h,f);if(r instanceof ra)return"return"===r.g?r.s:r}}())},list:function(a){var b=this.g.B;b.nb(arguments.length);for(var c=new k,d=0;d<arguments.length;d++){var e=F(this,arguments[d]);"string"===typeof e&&b.nb(e.length?e.length-1:0);c.push(e)}return c},map:function(a){for(var b=this.g.B,c=new tb,d=0;d<arguments.length-1;d+=2){var e=F(this,arguments[d])+"",f=F(this,arguments[d+1]),g=e.length;g+="string"===
typeof f?f.length:1;b.nb(g);c.set(e,f)}return c},undefined:function(){}};var wb=function(){this.B=za();this.g=new Aa(this.B)},xb=function(a,b,c){var d=new pb(b,c);d.mb();a.g.set(b,d)},yb=function(a,b,c){vb.hasOwnProperty(b)&&xb(a,c||b,vb[b])};wb.prototype.Gb=function(a,b){var c=Array.prototype.slice.call(arguments,0);return this.s(c)};wb.prototype.s=function(a){for(var b,c=0;c<arguments.length;c++)b=rb(this.g,arguments[c]);return b};wb.prototype.F=function(a,b){var c=Da(this.g);c.g=a;for(var d,e=1;e<arguments.length;e++)d=d=rb(c,arguments[e]);return d};var zb,Bb=function(){if(void 0===zb){var a=null,b=pa.trustedTypes;if(b&&b.createPolicy){try{a=b.createPolicy("goog#html",{createHTML:qa,createScript:qa,createScriptURL:qa})}catch(c){pa.console&&pa.console.error(c.message)}zb=a}else zb=a}return zb};var Db=function(a,b){this.g=b===Cb?a:""};Db.prototype.toString=function(){return this.g+""};var Cb={},Eb=function(a){var b=Bb(),c=b?b.createScriptURL(a):a;return new Db(c,Cb)};var Fb=/^(?:(?:https?|mailto|ftp):|[^:/?#]*(?:[/?#]|$))/i;var Gb;a:{var Hb=pa.navigator;if(Hb){var Ib=Hb.userAgent;if(Ib){Gb=Ib;break a}}Gb=""}var Jb=function(a){return-1!=Gb.indexOf(a)};var Lb=function(a,b,c){this.g=c===Kb?a:""};Lb.prototype.toString=function(){return this.g.toString()};var Mb=function(a){return a instanceof Lb&&a.constructor===Lb?a.g:"type_error:SafeHtml"},Kb={},Nb=function(a){var b=Bb(),c=b?b.createHTML(a):a;return new Lb(c,null,Kb)},Ob=new Lb(pa.trustedTypes&&pa.trustedTypes.emptyHTML||"",0,Kb);var Tb=function(a,b){a.src=b instanceof Db&&b.constructor===Db?b.g:"type_error:TrustedResourceUrl";var c,d,e=(a.ownerDocument&&a.ownerDocument.defaultView||window).document,f=null===(d=e.querySelector)||void 0===d?void 0:d.call(e,"script[nonce]");(c=f?f.nonce||f.getAttribute("nonce")||"":"")&&a.setAttribute("nonce",c)};var Ub=function(a,b){var c=function(){};c.prototype=a.prototype;var d=new c;a.apply(d,Array.prototype.slice.call(arguments,1));return d},Vb=function(a){var b=a;return function(){if(b){var c=b;b=null;c()}}};var Wb=function(a){var b=!1,c;return function(){b||(c=a(),b=!0);return c}}(function(){var a=document.createElement("div"),b=document.createElement("div");b.appendChild(document.createElement("div"));a.appendChild(b);var c=a.firstChild.firstChild;a.innerHTML=Mb(Ob);return!c.parentElement}),Xb=function(a,b){if(Wb())for(;a.lastChild;)a.removeChild(a.lastChild);a.innerHTML=Mb(b)};var A=window,H=document,Yb=navigator,Zb=H.currentScript&&H.currentScript.src,$b=function(a,b){var c=A[a];A[a]=void 0===c?b:c;return A[a]},ac=function(a){var b=H.getElementsByTagName("script")[0]||H.body||H.head;b.parentNode.insertBefore(a,b)},bc=function(a,b){b&&(a.addEventListener?a.onload=b:a.onreadystatechange=function(){a.readyState in{loaded:1,complete:1}&&(a.onreadystatechange=null,b())})},cc={async:1,nonce:1,onerror:1,onload:1,src:1,type:1},dc=function(a,b,c,d){var e=H.createElement("script");
d&&Ua(d,function(f,g){f=f.toLowerCase();cc.hasOwnProperty(f)||e.setAttribute(f,g)});e.type="text/javascript";e.async=!0;Tb(e,Eb(a));bc(e,b);c&&(e.onerror=c);ac(e);return e},ec=function(){if(Zb){var a=Zb.toLowerCase();if(0===a.indexOf("https://"))return 2;if(0===a.indexOf("http://"))return 3}return 1},fc=function(a,b){var c=H.createElement("iframe");c.height="0";c.width="0";c.style.display="none";c.style.visibility="hidden";var d=H.body&&H.body.lastChild||H.body||H.head;d.parentNode.insertBefore(c,
d);bc(c,b);void 0!==a&&(c.src=a);return c},gc=function(a,b,c){var d=new Image(1,1);d.onload=function(){d.onload=null;b&&b()};d.onerror=function(){d.onerror=null;c&&c()};d.src=a;return d},hc=function(a,b,c,d){a.addEventListener?a.addEventListener(b,c,!!d):a.attachEvent&&a.attachEvent("on"+b,c)},jc=function(a,b,c){a.removeEventListener?a.removeEventListener(b,c,!1):a.detachEvent&&a.detachEvent("on"+b,c)},J=function(a){A.setTimeout(a,0)},kc=function(a,b){return a&&b&&a.attributes&&a.attributes[b]?a.attributes[b].value:
null},lc=function(a){var b=a.innerText||a.textContent||"";b&&" "!=b&&(b=b.replace(/^[\s\xa0]+|[\s\xa0]+$/g,""));b&&(b=b.replace(/(\xa0+|\s{2,}|\n|\r\t)/g," "));return b},mc=function(a){var b=H.createElement("div"),c=Nb("A<div>"+a+"</div>");Xb(b,c);b=b.lastChild;for(var d=[];b.firstChild;)d.push(b.removeChild(b.firstChild));return d},nc=function(a,b,c){c=c||100;for(var d={},e=0;e<b.length;e++)d[b[e]]=!0;for(var f=a,g=0;f&&g<=c;g++){if(d[String(f.tagName).toLowerCase()])return f;f=f.parentElement}return null},
oc=function(a){Yb.sendBeacon&&Yb.sendBeacon(a)||gc(a)},pc=function(a,b){var c=a[b];c&&"string"===typeof c.animVal&&(c=c.animVal);return c};var qc=function(a,b){return F(this,a)&&F(this,b)},rc=function(a,b){return F(this,a)===F(this,b)},sc=function(a,b){return F(this,a)||F(this,b)},tc=function(a,b){a=F(this,a);b=F(this,b);return-1<String(a).indexOf(String(b))},uc=function(a,b){a=String(F(this,a));b=String(F(this,b));return a.substring(0,b.length)===b},vc=function(a,b){a=F(this,a);b=F(this,b);switch(a){case "pageLocation":var c=A.location.href;b instanceof tb&&b.get("stripProtocol")&&(c=c.replace(/^https?:\/\//,""));return c}};var xc=function(){this.g=new wb;wc(this)};xc.prototype.Gb=function(a){return this.g.s(a)};var wc=function(a){yb(a.g,"map");var b=function(c,d){xb(a.g,c,d)};b("and",qc);b("contains",tc);b("equals",rc);b("or",sc);b("startsWith",uc);b("variable",vc)};var zc=function(a){if(a instanceof zc)return a;this.Na=a};zc.prototype.toString=function(){return String(this.Na)};/*
 jQuery v1.9.1 (c) 2005, 2012 jQuery Foundation, Inc. jquery.org/license. */
var Ac=/\[object (Boolean|Number|String|Function|Array|Date|RegExp)\]/,Bc=function(a){if(null==a)return String(a);var b=Ac.exec(Object.prototype.toString.call(Object(a)));return b?b[1].toLowerCase():"object"},Cc=function(a,b){return Object.prototype.hasOwnProperty.call(Object(a),b)},Dc=function(a){if(!a||"object"!=Bc(a)||a.nodeType||a==a.window)return!1;try{if(a.constructor&&!Cc(a,"constructor")&&!Cc(a.constructor.prototype,"isPrototypeOf"))return!1}catch(c){return!1}for(var b in a);return void 0===
b||Cc(a,b)},L=function(a,b){var c=b||("array"==Bc(a)?[]:{}),d;for(d in a)if(Cc(a,d)){var e=a[d];"array"==Bc(e)?("array"!=Bc(c[d])&&(c[d]=[]),c[d]=L(e,c[d])):Dc(e)?(Dc(c[d])||(c[d]={}),c[d]=L(e,c[d])):c[d]=e}return c};var Fc=function(a,b,c){var d=[],e=[],f=function(h,l){for(var m=va(h,1),p=0;p<m.length;p++)l[m[p]]=g(h.get(m[p]))},g=function(h){var l=Oa(d,h);if(-1<l)return e[l];if(h instanceof k){var m=[];d.push(h);e.push(m);for(var p=h.lb(),q=0;q<p.length();q++)m[p.get(q)]=g(h.get(p.get(q)));return m}if(h instanceof tb){var r={};d.push(h);e.push(r);f(h,r);return r}if(h instanceof pb){var t=function(){for(var u=Array.prototype.slice.call(arguments,0),v=0;v<u.length;v++)u[v]=Ec(u[v],b,!!c);var w=b?b.B:za(),y=new Aa(w);
b&&(y.g=b.g);return g(h.g.apply(h,[y].concat(u)))};d.push(h);e.push(t);f(h,t);return t}switch(typeof h){case "boolean":case "number":case "string":case "undefined":return h;case "object":if(null===h)return null}};return g(a)},Ec=function(a,b,c){var d=[],e=[],f=function(h,l){for(var m in h)h.hasOwnProperty(m)&&l.set(m,g(h[m]))},g=function(h){var l=Oa(d,
h);if(-1<l)return e[l];if(Na(h)||Va(h)){var m=new k([]);d.push(h);e.push(m);for(var p in h)h.hasOwnProperty(p)&&m.set(p,g(h[p]));return m}if(Dc(h)){var q=new tb;d.push(h);e.push(q);f(h,q);return q}if("function"===typeof h){var r=new pb("",function(u){for(var v=Array.prototype.slice.call(arguments,0),w=0;w<v.length;w++)v[w]=Fc(F(this,v[w]),b,!!c);return g((0,this.g.P)(h,h,v))});d.push(h);e.push(r);f(h,r);return r}var t=typeof h;if(null===h||"string"===t||"number"===t||"boolean"===t)return h;};return g(a)};var Gc=function(a){for(var b=[],c=0;c<a.length();c++)a.has(c)&&(b[c]=a.get(c));return b},Hc=function(a){if(void 0===a||Na(a)||Dc(a))return!0;switch(typeof a){case "boolean":case "number":case "string":case "function":return!0}return!1};var Ic={supportedMethods:"concat every filter forEach hasOwnProperty indexOf join lastIndexOf map pop push reduce reduceRight reverse shift slice some sort splice unshift toString".split(" "),concat:function(a,b){for(var c=[],d=0;d<this.length();d++)c.push(this.get(d));for(var e=1;e<arguments.length;e++)if(arguments[e]instanceof k)for(var f=arguments[e],g=0;g<f.length();g++)c.push(f.get(g));else c.push(arguments[e]);return new k(c)},every:function(a,b){for(var c=this.length(),d=0;d<this.length()&&
d<c;d++)if(this.has(d)&&!b.g(a,this.get(d),d,this))return!1;return!0},filter:function(a,b){for(var c=this.length(),d=[],e=0;e<this.length()&&e<c;e++)this.has(e)&&b.g(a,this.get(e),e,this)&&d.push(this.get(e));return new k(d)},forEach:function(a,b){for(var c=this.length(),d=0;d<this.length()&&d<c;d++)this.has(d)&&b.g(a,this.get(d),d,this)},hasOwnProperty:function(a,b){return this.has(b)},indexOf:function(a,b,c){var d=this.length(),e=void 0===c?0:Number(c);0>e&&(e=Math.max(d+e,0));for(var f=e;f<d;f++)if(this.has(f)&&
this.get(f)===b)return f;return-1},join:function(a,b){for(var c=[],d=0;d<this.length();d++)c.push(this.get(d));return c.join(b)},lastIndexOf:function(a,b,c){var d=this.length(),e=d-1;void 0!==c&&(e=0>c?d+c:Math.min(c,e));for(var f=e;0<=f;f--)if(this.has(f)&&this.get(f)===b)return f;return-1},map:function(a,b){for(var c=this.length(),d=[],e=0;e<this.length()&&e<c;e++)this.has(e)&&(d[e]=b.g(a,this.get(e),e,this));return new k(d)},pop:function(){return this.pop()},push:function(a,b){return this.push.apply(this,
Array.prototype.slice.call(arguments,1))},reduce:function(a,b,c){var d=this.length(),e,f=0;if(void 0!==c)e=c;else{if(0===d)throw Error("TypeError: Reduce on List with no elements.");for(var g=0;g<d;g++)if(this.has(g)){e=this.get(g);f=g+1;break}if(g===d)throw Error("TypeError: Reduce on List with no elements.");}for(var h=f;h<d;h++)this.has(h)&&(e=b.g(a,e,this.get(h),h,this));return e},reduceRight:function(a,b,c){var d=this.length(),e,f=d-1;if(void 0!==c)e=c;else{if(0===d)throw Error("TypeError: ReduceRight on List with no elements.");
for(var g=1;g<=d;g++)if(this.has(d-g)){e=this.get(d-g);f=d-(g+1);break}if(g>d)throw Error("TypeError: ReduceRight on List with no elements.");}for(var h=f;0<=h;h--)this.has(h)&&(e=b.g(a,e,this.get(h),h,this));return e},reverse:function(){for(var a=Gc(this),b=a.length-1,c=0;0<=b;b--,c++)a.hasOwnProperty(b)?this.set(c,a[b]):xa(this,c);return this},shift:function(){return this.shift()},slice:function(a,b,c){var d=this.length();void 0===b&&(b=0);b=0>b?Math.max(d+b,0):Math.min(b,d);c=void 0===c?d:0>c?
Math.max(d+c,0):Math.min(c,d);c=Math.max(b,c);for(var e=[],f=b;f<c;f++)e.push(this.get(f));return new k(e)},some:function(a,b){for(var c=this.length(),d=0;d<this.length()&&d<c;d++)if(this.has(d)&&b.g(a,this.get(d),d,this))return!0;return!1},sort:function(a,b){var c=Gc(this);void 0===b?c.sort():c.sort(function(e,f){return Number(b.g(a,e,f))});for(var d=0;d<c.length;d++)c.hasOwnProperty(d)?this.set(d,c[d]):xa(this,d)},splice:function(a,b,c,d){return this.splice.apply(this,Array.prototype.splice.call(arguments,
1,arguments.length-1))},toString:function(){return this.toString()},unshift:function(a,b){return this.unshift.apply(this,Array.prototype.slice.call(arguments,1))}};var Mc="charAt concat indexOf lastIndexOf match replace search slice split substring toLowerCase toLocaleLowerCase toString toUpperCase toLocaleUpperCase trim".split(" "),Nc=new ra("break"),Oc=new ra("continue"),Pc=function(a,b){return F(this,a)+F(this,b)},Qc=function(a,b){return F(this,a)&&F(this,b)},Rc=function(a,b,c){a=F(this,a);b=F(this,b);c=F(this,c);if(!(c instanceof k))throw Error("Error: Non-List argument given to Apply instruction.");if(null===a||void 0===a)throw Error("TypeError: Can't read property "+
b+" of "+a+".");if("boolean"===typeof a||"number"===typeof a){if("toString"===b)return a.toString();throw Error("TypeError: "+a+"."+b+" is not a function.");}if("string"===typeof a){if(0<=Oa(Mc,b)){var d=Fc(c);return Ec(a[b].apply(a,d),this.g)}throw Error("TypeError: "+b+" is not a function");}if(a instanceof k){if(a.has(b)){var e=a.get(b);if(e instanceof pb){var f=Gc(c);f.unshift(this.g);return e.g.apply(e,f)}throw Error("TypeError: "+b+" is not a function");}if(0<=Oa(Ic.supportedMethods,b)){var g=
Gc(c);g.unshift(this.g);return Ic[b].apply(a,g)}}if(a instanceof pb||a instanceof tb){if(a.has(b)){var h=a.get(b);if(h instanceof pb){var l=Gc(c);l.unshift(this.g);return h.g.apply(h,l)}throw Error("TypeError: "+b+" is not a function");}if("toString"===b)return a instanceof pb?a.P:a.toString();if("hasOwnProperty"===b)return a.has.apply(a,Gc(c))}if(a instanceof zc&&"toString"===b)return a.toString();throw Error("TypeError: Object has no '"+b+"' property.");},Sc=function(a,b){a=F(this,a);if("string"!==
typeof a)throw Error("Invalid key name given for assignment.");var c=this.g;if(!c.has(a))throw Error("Attempting to assign to undefined value "+b);var d=F(this,b);c.set(a,d);return d},Tc=function(a){var b=Da(this.g),c=sb(b,Array.prototype.slice.apply(arguments));if(c instanceof ra)return c},Uc=function(){return Nc},Vc=function(a){for(var b=F(this,a),c=0;c<b.length;c++){var d=F(this,b[c]);if(d instanceof ra)return d}},Wc=function(a){for(var b=this.g,c=0;c<arguments.length-1;c+=2){var d=arguments[c];
if("string"===typeof d){var e=F(this,arguments[c+1]);Ba(b,d,e,!0)}}},Xc=function(){return Oc},Yc=function(a,b,c){var d=new k;b=F(this,b);for(var e=0;e<b.length;e++)d.push(b[e]);var f=[51,a,d].concat(Array.prototype.splice.call(arguments,2,arguments.length-2));this.g.add(a,F(this,f))},Zc=function(a,b){return F(this,a)/F(this,b)},$c=function(a,b){a=F(this,a);b=F(this,b);var c=a instanceof zc,d=b instanceof zc;return c||d?c&&d?a.Na==b.Na:!1:a==b},ad=function(a){for(var b,c=0;c<arguments.length;c++)b=
F(this,arguments[c]);return b};function bd(a,b,c,d){for(var e=0;e<b();e++){var f=a(c(e)),g=sb(f,d);if(g instanceof ra){if("break"===g.g)break;if("return"===g.g)return g}}}function cd(a,b,c){if("string"===typeof b)return bd(a,function(){return b.length},function(f){return f},c);if(b instanceof tb||b instanceof k||b instanceof pb){var d=b.lb(),e=d.length();return bd(a,function(){return e},function(f){return d.get(f)},c)}}
var dd=function(a,b,c){a=F(this,a);b=F(this,b);c=F(this,c);var d=this.g;return cd(function(e){d.set(a,e);return d},b,c)},ed=function(a,b,c){a=F(this,a);b=F(this,b);c=F(this,c);var d=this.g;return cd(function(e){var f=Da(d);Ba(f,a,e,!0);return f},b,c)},fd=function(a,b,c){a=F(this,a);b=F(this,b);c=F(this,c);var d=this.g;return cd(function(e){var f=Da(d);f.add(a,e);return f},b,c)},hd=function(a,b,c){a=F(this,a);b=F(this,b);c=F(this,c);var d=this.g;return gd(function(e){d.set(a,e);return d},b,c)},id=
function(a,b,c){a=F(this,a);b=F(this,b);c=F(this,c);var d=this.g;return gd(function(e){var f=Da(d);Ba(f,a,e,!0);return f},b,c)},jd=function(a,b,c){a=F(this,a);b=F(this,b);c=F(this,c);var d=this.g;return gd(function(e){var f=Da(d);f.add(a,e);return f},b,c)};
function gd(a,b,c){if("string"===typeof b)return bd(a,function(){return b.length},function(d){return b[d]},c);if(b instanceof k)return bd(a,function(){return b.length()},function(d){return b.get(d)},c);throw new TypeError("The value is not iterable.");}
var kd=function(a,b,c,d){function e(p,q){for(var r=0;r<f.length();r++){var t=f.get(r);q.add(t,p.get(t))}}var f=F(this,a);if(!(f instanceof k))throw Error("TypeError: Non-List argument given to ForLet instruction.");var g=this.g;d=F(this,d);var h=Da(g);for(e(g,h);rb(h,b);){var l=sb(h,d);if(l instanceof ra){if("break"===l.g)break;if("return"===l.g)return l}var m=Da(g);e(h,m);rb(m,c);h=m}},ld=function(a){a=F(this,a);var b=this.g,c=!1;if(c&&!b.has(a))throw new ReferenceError(a+" is not defined.");return b.get(a)},sd=function(a,b){var c;a=F(this,a);b=F(this,b);if(void 0===a||null===a)throw Error("TypeError: cannot access property of "+a+".");if(a instanceof tb||a instanceof k||a instanceof pb)c=a.get(b);else if("string"===typeof a)"length"===b?c=a.length:sa(b)&&(c=a[b]);else if(a instanceof zc)return;return c},td=function(a,b){return F(this,a)>F(this,
b)},ud=function(a,b){return F(this,a)>=F(this,b)},vd=function(a,b){a=F(this,a);b=F(this,b);a instanceof zc&&(a=a.Na);b instanceof zc&&(b=b.Na);return a===b},wd=function(a,b){return!vd.call(this,a,b)},xd=function(a,b,c){var d=[];F(this,a)?d=F(this,b):c&&(d=F(this,c));var e=sb(this.g,d);if(e instanceof ra)return e},yd=function(a,b){return F(this,a)<F(this,b)},zd=function(a,b){return F(this,a)<=F(this,b)},Ad=function(a,b){return F(this,a)%F(this,b)},Bd=function(a,b){return F(this,a)*F(this,b)},Cd=function(a){return-F(this,
a)},Dd=function(a){return!F(this,a)},Ed=function(a,b){return!$c.call(this,a,b)},Fd=function(){return null},Gd=function(a,b){return F(this,a)||F(this,b)},Hd=function(a,b){var c=F(this,a);F(this,b);return c},Id=function(a){return F(this,a)},Jd=function(a){return Array.prototype.slice.apply(arguments)},Kd=function(a){return new ra("return",F(this,a))},Ld=function(a,b,c){a=F(this,a);b=F(this,b);c=F(this,c);if(null===a||void 0===a)throw Error("TypeError: Can't set property "+b+" of "+a+".");(a instanceof
pb||a instanceof k||a instanceof tb)&&a.set(b,c);return c},Md=function(a,b){return F(this,a)-F(this,b)},Nd=function(a,b,c){a=F(this,a);var d=F(this,b),e=F(this,c);if(!Na(d)||!Na(e))throw Error("Error: Malformed switch instruction.");for(var f,g=!1,h=0;h<d.length;h++)if(g||a===F(this,d[h]))if(f=F(this,e[h]),f instanceof ra){var l=f.g;if("break"===l)return;if("return"===l||"continue"===l)return f}else g=!0;if(e.length===d.length+1&&(f=F(this,e[e.length-1]),f instanceof ra&&("return"===f.g||"continue"===
f.g)))return f},Od=function(a,b,c){return F(this,a)?F(this,b):F(this,c)},Pd=function(a){a=F(this,a);return a instanceof pb?"function":typeof a},Qd=function(a){for(var b=this.g,c=0;c<arguments.length;c++){var d=arguments[c];"string"!==typeof d||b.add(d,void 0)}},Rd=function(a,b,c,d){var e=F(this,d);if(F(this,c)){var f=sb(this.g,e);if(f instanceof ra){if("break"===f.g)return;if("return"===f.g)return f}}for(;F(this,a);){var g=sb(this.g,e);if(g instanceof ra){if("break"===g.g)break;if("return"===g.g)return g}F(this,
b)}},Sd=function(a){return~Number(F(this,a))},Td=function(a,b){return Number(F(this,a))<<Number(F(this,b))},Ud=function(a,b){return Number(F(this,a))>>Number(F(this,b))},Vd=function(a,b){return Number(F(this,a))>>>Number(F(this,b))},Wd=function(a,b){return Number(F(this,a))&Number(F(this,b))},Xd=function(a,b){return Number(F(this,a))^Number(F(this,b))},Yd=function(a,b){return Number(F(this,a))|Number(F(this,b))};var ge=function(){this.g=new wb;Zd(this)};ge.prototype.Gb=function(a){return he(this.g.s(a))};
var je=function(a,b){return he(ie.g.F(a,b))},Zd=function(a){var b=function(d,e){yb(a.g,d,String(e))};b("control",49);b("fn",51);b("list",7);b("map",8);b("undefined",44);var c=function(d,e){xb(a.g,String(d),e)};c(0,Pc);c(1,Qc);c(2,Rc);c(3,Sc);c(53,Tc);c(4,Uc);c(5,Vc);c(52,Wc);c(6,Xc);c(9,Vc);c(50,Yc);c(10,Zc);c(12,$c);c(13,ad);c(47,dd);c(54,ed);c(55,fd);c(63,kd);c(64,hd);c(65,id);c(66,jd);c(15,ld);c(16,sd);c(17,sd);c(18,td);c(19,ud);c(20,vd);c(21,wd);c(22,xd);c(23,yd);c(24,zd);c(25,Ad);c(26,Bd);c(27,
Cd);c(28,Dd);c(29,Ed);c(45,Fd);c(30,Gd);c(32,Hd);c(33,Hd);c(34,Id);c(35,Id);c(46,Jd);c(36,Kd);c(43,Ld);c(37,Md);c(38,Nd);c(39,Od);c(40,Pd);c(41,Qd);c(42,Rd);c(58,Sd);c(57,Td);c(60,Ud);c(61,Vd);c(56,Wd);c(62,Xd);c(59,Yd)},le=function(){var a=ie,b=ke();xb(a.g,"require",b)},me=function(a,b){a.g.g.P=b};function he(a){if(a instanceof ra||a instanceof pb||a instanceof k||a instanceof tb||a instanceof zc||null===a||void 0===a||"string"===typeof a||"number"===typeof a||"boolean"===typeof a)return a};var ne=function(){var a=function(b){return{toString:function(){return b}}};return{Ng:a("consent"),zd:a("consent_always_fire"),ef:a("convert_case_to"),ff:a("convert_false_to"),hf:a("convert_null_to"),jf:a("convert_true_to"),kf:a("convert_undefined_to"),aj:a("debug_mode_metadata"),kb:a("function"),yh:a("instance_name"),Ah:a("live_only"),Bh:a("malware_disabled"),Ch:a("metadata"),dj:a("original_activity_id"),ej:a("original_vendor_template_id"),Eh:a("once_per_event"),Of:a("once_per_load"),gj:a("priority_override"),
ij:a("respected_consent_types"),Tf:a("setup_tags"),Vf:a("tag_id"),Wf:a("teardown_tags")}}();var Je;
var Ke=[],Le=[],Me=[],Ne=[],Oe=[],Pe={},Qe,Re,Se,Te=function(a,b){var c={};c["function"]="__"+a;for(var d in b)b.hasOwnProperty(d)&&(c["vtp_"+d]=b[d]);return c},Ue=function(a,b){var c=a["function"];if(!c)throw Error("Error: No function name given for function call.");var d=Pe[c],e={},f;for(f in a)if(a.hasOwnProperty(f))if(0===f.indexOf("vtp_"))d&&b&&b.dg&&b.dg(a[f]),e[void 0!==d?f:f.substr(4)]=a[f];else if(f===ne.zd.toString()&&a[f]){}d&&b&&b.cg&&(e.vtp_gtmCachedValues=b.cg);return void 0!==d?d(e):Je(c,e,b)},We=function(a,b,c){c=c||[];var d={},e;for(e in a)a.hasOwnProperty(e)&&(d[e]=Ve(a[e],b,c));return d},Ve=function(a,b,c){if(Na(a)){var d;switch(a[0]){case "function_id":return a[1];case "list":d=[];for(var e=1;e<a.length;e++)d.push(Ve(a[e],b,c));return d;case "macro":var f=
a[1];if(c[f])return;var g=Ke[f];if(!g||b.Ge(g))return;c[f]=!0;try{var h=We(g,b,c);h.vtp_gtmEventId=b.id;d=Ue(h,b);Se&&(d=Se.Th(d,h))}catch(y){b.rg&&b.rg(y,Number(f)),d=!1}c[f]=!1;return d;case "map":d={};for(var l=1;l<a.length;l+=2)d[Ve(a[l],b,c)]=Ve(a[l+1],b,c);return d;case "template":d=[];for(var m=!1,p=1;p<a.length;p++){var q=Ve(a[p],b,c);Re&&(m=m||q===Re.Xc);d.push(q)}return Re&&m?Re.Wh(d):d.join("");case "escape":d=Ve(a[1],b,c);if(Re&&Na(a[1])&&"macro"===a[1][0]&&Re.ni(a))return Re.Hi(d);d=
String(d);for(var r=2;r<a.length;r++)oe[a[r]]&&(d=oe[a[r]](d));return d;case "tag":var t=a[1];if(!Ne[t])throw Error("Unable to resolve tag reference "+t+".");return d={lg:a[2],index:t};case "zb":var u={arg0:a[2],arg1:a[3],ignore_case:a[5]};u["function"]=a[1];var v=Xe(u,b,c),w=!!a[4];return w||2!==v?w!==(1===v):null;default:throw Error("Attempting to expand unknown Value type: "+a[0]+".");}}return a},Xe=function(a,b,c){try{return Qe(We(a,b,c))}catch(d){JSON.stringify(a)}return 2};var Ye=function(a,b,c){var d;d=Error.call(this);this.message=d.message;"stack"in d&&(this.stack=d.stack);this.s=a;this.g=c};oa(Ye,Error);function Ze(a,b){if(Na(a)){Object.defineProperty(a,"context",{value:{line:b[0]}});for(var c=1;c<a.length;c++)Ze(a[c],b[c])}};var $e=function(a,b){var c;c=Error.call(this);this.message=c.message;"stack"in c&&(this.stack=c.stack);this.B=a;this.s=b;this.g=[]};oa($e,Error);var bf=function(){return function(a,b){a instanceof $e||(a=new $e(a,af));b&&a.g.push(b);throw a;}};function af(a){if(!a.length)return a;a.push({id:"main",line:0});for(var b=a.length-1;0<b;b--)La(a[b].id)&&a.splice(b++,1);for(var c=a.length-1;0<c;c--)a[c].line=a[c-1].line;a.splice(0,1);return a};var ef=function(a){function b(r){for(var t=0;t<r.length;t++)d[r[t]]=!0}for(var c=[],d=[],e=cf(a),f=0;f<Le.length;f++){var g=Le[f],h=df(g,e);if(h){for(var l=g.add||[],m=0;m<l.length;m++)c[l[m]]=!0;b(g.block||[])}else null===h&&b(g.block||[]);}for(var p=[],q=0;q<Ne.length;q++)c[q]&&!d[q]&&(p[q]=!0);return p},df=function(a,b){for(var c=a["if"]||[],d=0;d<c.length;d++){var e=b(c[d]);if(0===e)return!1;if(2===e)return null}for(var f=
a.unless||[],g=0;g<f.length;g++){var h=b(f[g]);if(2===h)return null;if(1===h)return!1}return!0},cf=function(a){var b=[];return function(c){void 0===b[c]&&(b[c]=Xe(Me[c],a));return b[c]}};var ff={Th:function(a,b){b[ne.ef]&&"string"===typeof a&&(a=1==b[ne.ef]?a.toLowerCase():a.toUpperCase());b.hasOwnProperty(ne.hf)&&null===a&&(a=b[ne.hf]);b.hasOwnProperty(ne.kf)&&void 0===a&&(a=b[ne.kf]);b.hasOwnProperty(ne.jf)&&!0===a&&(a=b[ne.jf]);b.hasOwnProperty(ne.ff)&&!1===a&&(a=b[ne.ff]);return a}};var gf=function(){this.g={}};function hf(a,b,c,d){if(a)for(var e=0;e<a.length;e++){var f=void 0,g="A policy function denied the permission request";try{f=a[e].call(void 0,b,c,d),g+="."}catch(h){g="string"===typeof h?g+(": "+h):h instanceof Error?g+(": "+h.message):g+"."}if(!f)throw new Ye(c,d,g);}}function jf(a,b,c){return function(){var d=arguments[0];if(d){var e=a.g[d],f=a.g.all;if(e||f){var g=c.apply(void 0,Array.prototype.slice.call(arguments,0));hf(e,b,d,g);hf(f,b,d,g)}}}};var of=function(a){var b=kf.M,c=this;this.s=new gf;this.g={};var d={},e=jf(this.s,b,function(){var f=arguments[0];return f&&d[f]?d[f].apply(void 0,Array.prototype.slice.call(arguments,0)):{}});Ua(a,function(f,g){var h={};Ua(g,function(l,m){var p=mf(l,m);h[l]=p.assert;d[l]||(d[l]=p.T)});c.g[f]=function(l,m){var p=h[l];if(!p)throw nf(l,{},"The requested permission "+l+" is not configured.");var q=Array.prototype.slice.call(arguments,0);p.apply(void 0,q);e.apply(void 0,q)}})},qf=function(a){return pf.g[a]||
function(){}};function mf(a,b){var c=Te(a,b);c.vtp_permissionName=a;c.vtp_createPermissionError=nf;try{return Ue(c)}catch(d){return{assert:function(e){throw new Ye(e,{},"Permission "+e+" is unknown.");},T:function(){for(var e={},f=0;f<arguments.length;++f)e["arg"+(f+1)]=arguments[f];return e}}}}function nf(a,b,c){return new Ye(a,b,c)};var rf=!1;var sf={};sf.$i=Xa('false');sf.Zh=Xa('true');var tf=rf,uf=sf.Zh,vf=sf.$i;
var Kf=function(a,b){return a.length&&b.length&&a.lastIndexOf(b)===a.length-b.length},Lf=function(a,b){var c="*"===b.charAt(b.length-1)||"/"===b||"/*"===b;Kf(b,"/*")&&(b=b.slice(0,-2));Kf(b,"?")&&(b=b.slice(0,-1));var d=b.split("*");if(!c&&1===d.length)return a===d[0];for(var e=-1,f=0;f<d.length;f++){var g=d[f];if(g){e=a.indexOf(g,e);if(-1===e||0===f&&0!==e)return!1;e+=g.length}}if(c||e===a.length)return!0;var h=d[d.length-1];return a.lastIndexOf(h)===a.length-h.length},Mf=/^[a-z0-9-]+$/i,Nf=/^https:\/\/(\*\.|)((?:[a-z0-9-]+\.)+[a-z0-9-]+)\/(.*)$/i,
Pf=function(a,b){var c;if(!(c=!Of(a))){var d;a:{var e=a.hostname.split(".");if(2>e.length)d=!1;else{for(var f=0;f<e.length;f++)if(!Mf.exec(e[f])){d=!1;break a}d=!0}}c=!d}if(c)return!1;for(var g=0;g<b.length;g++){var h;var l=a,m=b[g];if(!Nf.exec(m))throw Error("Invalid Wildcard");var p=m.slice(8),q=p.slice(0,p.indexOf("/")),r;var t=l.hostname,u=q;if(0!==u.indexOf("*."))r=t.toLowerCase()===u.toLowerCase();else{u=u.slice(2);var v=t.toLowerCase().indexOf(u.toLowerCase());r=-1===v?!1:t.length===u.length?
!0:t.length!==u.length+v?!1:"."===t[v-1]}if(r){var w=p.slice(p.indexOf("/"));h=Lf(l.pathname+l.search,w)?!0:!1}else h=!1;if(h)return!0}return!1},Of=function(a){return"https:"===a.protocol&&(!a.port||"443"===a.port)};var Qf=/^([a-z][a-z0-9]*):(!|\?)(\*|string|boolean|number|Fn|DustMap|List|OpaqueValue)$/i,Rf={Fn:"function",DustMap:"Object",List:"Array"},M=function(a,b,c){for(var d=0;d<b.length;d++){var e=Qf.exec(b[d]);if(!e)throw Error("Internal Error in "+a);var f=e[1],g="!"===e[2],h=e[3],l=c[d],m=typeof l;if(null===l||"undefined"===m){if(g)throw Error("Error in "+a+". Required argument "+f+" not supplied.");}else if("*"!==h){var p=typeof l;l instanceof pb?p="Fn":l instanceof k?p="List":l instanceof tb?p="DustMap":
l instanceof zc&&(p="OpaqueValue");if(p!=h)throw Error("Error in "+a+". Argument "+f+" has type "+p+", which does not match required type "+(Rf[h]||h)+".");}}};function Sf(a){if(a instanceof tb)return a.toString();if(a instanceof k)return"["+a.toString()+"]";if(a instanceof pb)return a.toString()+"()";if(n(a))return'"'+a+'"';return""+a}
function Tf(a,b){var c=[];var d=[],e=[],f=function(){return 0===e.length?"":"Property "+e.join(".")+": "},g=function(h,l){if(0<=d.indexOf(l))c.push(f()+"Expected value contained a cycle.");else if(h!==l)if(l instanceof k)if(h instanceof k)if(h.length()!==l.length())c.push(f()+"Expected array to contain "+l.length()+" item(s), actually "+h.length()+".");else for(var m=0;m<l.length();m++)e.push(m),d.push(l),g(h.get(m),l.get(m)),d.pop(),e.pop();else c.push(f()+"Expected to be an array, actually "+
Sf(h)+".");else if(l instanceof tb)if(h instanceof tb){for(var p=va(h,1),q={},r=0;r<p.length;r++)q[p[r]]=!0;for(var t=va(l,1),u=[],v=0;v<t.length;v++){var w=t[v];q[w]?(u.push(w),q[w]=!1):c.push(f()+'Expected property "'+w+'" was not found in actual.')}for(var y=0;y<p.length;y++)q[p[y]]&&c.push(f()+'Unexpected property "'+p[y]+'" found in actual.');for(var x=0;x<u.length;x++){var z=u[x];e.push(z);d.push(l);g(h.get(z),l.get(z));d.pop();e.pop()}}else c.push(f()+"Expected to be an object, actually "+
Sf(h)+".");else c.push(f()+"Expected "+Sf(l)+", actually "+Sf(h)+".")};g(a,b);return c};var Uf=function(a,b){var c=new pb(a,function(){for(var d=Array.prototype.slice.call(arguments,0),e=0;e<d.length;e++)d[e]=F(this,d[e]);var f=this.g.g;f&&f.wa&&(f.wa.cc[a]=f.wa.cc[a]||[],f.wa.cc[a].push(d));return b.apply(this,d)});c.mb();return c},Vf=function(a,b){var c=new tb,d;for(d in b)if(b.hasOwnProperty(d)){var e=b[d];Ja(e)?c.set(d,Uf(a+"_"+d,e)):(La(e)||n(e)||"boolean"==typeof e)&&c.set(d,e)}c.mb();return c};var Wf=function(a,b){M(G(this),["apiName:!string","message:?string"],arguments);var c={},d=new tb;return d=Vf("AssertApiSubject",c)};var Xf=function(a,b){M(G(this),["actual:?*","message:?string"],arguments);var c={},d=new tb;
return d=Vf("AssertThatSubject",c)};function Yf(a){return function(){for(var b=[],c=this.g,d=0;d<arguments.length;++d)b.push(Fc(arguments[d],c));return Ec(a.apply(null,b))}}var $f=function(){for(var a=Math,b=Zf,c={},d=0;d<b.length;d++){var e=b[d];a.hasOwnProperty(e)&&(c[e]=Yf(a[e].bind(a)))}return c};var ag=function(a){var b;return b};var bg=function(a){var b;return b};var cg=function(a){M(G(this),["uri:!string"],arguments);return encodeURI(a)};var dg=function(a){M(G(this),["uri:!string"],arguments);return encodeURIComponent(a)};var eg=function(a){M(G(this),["message:?string"],arguments);};var fg=function(a,b){M(G(this),["min:!number","max:!number"],arguments);return Ra(a,b)};var gg=function(a,b,c){var d=a.g.g;if(!d)throw Error("Missing program state.");d.Oh.apply(null,Array.prototype.slice.call(arguments,1))};var hg=function(){gg(this,"read_container_data");var a=new tb;a.set("containerId",'GTM-M6FZ495');a.set("version",'50');a.set("environmentName",'Live');a.set("debugMode",tf);a.set("previewMode",vf);a.set("environmentMode",uf);a.mb();return a};var ig=function(){return(new Date).getTime()};var jg=function(a){if(null===a)return"null";if(a instanceof k)return"array";if(a instanceof pb)return"function";if(a instanceof zc){a=a.Na;if(void 0===a.constructor||void 0===a.constructor.name){var b=String(a);return b.substring(8,b.length-1)}return String(a.constructor.name)}return typeof a};var kg=function(a){function b(c){return function(d){try{return c(d)}catch(e){(tf||vf)&&a.call(this,e.message)}}}return{parse:b(function(c){return Ec(JSON.parse(c))}),stringify:b(function(c){return JSON.stringify(Fc(c))})}};var lg=function(a){return Wa(Fc(a,this.g))};var mg=function(a){return Number(Fc(a,this.g))};var ng=function(a){return null===a?"null":void 0===a?"undefined":a.toString()};var og=function(a,b,c){var d=null,e=!1;return e?d:null};var Zf="floor ceil round max min abs pow sqrt".split(" ");var pg=function(){var a={};return{ei:function(b){return a.hasOwnProperty(b)?a[b]:void 0},Ui:function(b,c){a[b]=c},reset:function(){a={}}}},qg=function(a,b){M(G(this),["apiName:!string","mock:?*"],arguments);};var rg={};
rg.keys=function(a){return new k};
rg.values=function(a){return new k};
rg.entries=function(a){return new k};rg.freeze=function(a){return a};var tg=function(){this.g={};this.s={};};tg.prototype.get=function(a,b){var c=this.g.hasOwnProperty(a)?this.g[a]:void 0;c=ug(a,b)||c;return c};
tg.prototype.add=function(a,b,c){if(this.g.hasOwnProperty(a))throw"Attempting to add a function which already exists: "+a+".";if(this.s.hasOwnProperty(a))throw"Attempting to add an API with an existing private API name: "+a+".";this.g[a]=c?void 0:Ja(b)?Uf(a,b):Vf(a,b)};
var vg=function(a,b,c){if(a.s.hasOwnProperty(b))throw"Attempting to add a private function which already exists: "+b+".";if(a.g.hasOwnProperty(b))throw"Attempting to add a private function with an existing API name: "+b+".";a.s[b]=Ja(c)?Uf(b,c):Vf(b,c)};function ug(a,b){var c=void 0;var d=b.g.g;d&&d.wa&&(c=d.wa.Je.ei(a));return c};function wg(){var a={};return a};var N={Xb:"_ee",cd:"_syn_or_mod",jj:"_uei",Zd:"_eu",fj:"_pci",Qd:"event_callback",Lc:"event_timeout",xa:"gtag.config",Ja:"gtag.get",la:"purchase",xb:"refund",ab:"begin_checkout",vb:"add_to_cart",wb:"remove_from_cart",Wg:"view_cart",qf:"add_to_wishlist",Ia:"view_item",pf:"view_promotion",nf:"select_promotion",Cd:"select_item",Gc:"view_item_list",lf:"add_payment_info",Vg:"add_shipping_info",Wa:"value_key",Va:"value_callback",ya:"allow_ad_personalization_signals",Ub:"restricted_data_processing",Qb:"allow_google_signals",
Ba:"cookie_expires",Rb:"cookie_update",Wb:"session_duration",Qc:"session_engaged_time",Ma:"user_properties",na:"transport_url",R:"ads_data_redaction",Cb:"user_data",Sb:"first_party_collection",D:"ad_storage",H:"analytics_storage",Ad:"region",df:"wait_for_update",Aa:"conversion_linker",za:"conversion_cookie_prefix",ba:"value",aa:"currency",Hf:"trip_type",X:"items",Af:"passengers",Dd:"allow_custom_scripts",Bb:"session_id",Ff:"quantity",jb:"transaction_id",fb:"language",Kc:"country",Ic:"allow_enhanced_conversions",
Id:"aw_merchant_id",Gd:"aw_feed_country",Hd:"aw_feed_language",Fd:"discount",uf:"developer_id",Sc:"delivery_postal_code",Pd:"estimated_delivery_date",Md:"shipping",Xd:"new_customer",Jd:"customer_lifetime_value",Od:"enhanced_conversions"};N.Lf=[N.la,N.xb,N.ab,N.vb,N.wb,N.Wg,N.qf,N.Ia,N.pf,N.nf,N.Gc,N.Cd,N.lf,N.Vg];N.Kf=[N.ya,N.Qb,N.Rb];N.Mf=[N.Ba,N.Lc,N.Wb,N.Qc];var yg=function(a){Fa("GTM",a)};var zg=function(a,b){this.g=a;this.defaultValue=void 0===b?!1:b};var Ag=new zg(1936,!0),Bg=new zg(1933),Cg=new zg(373442741);var Eg=function(){var a=Dg;if(a.Ee&&a.hasOwnProperty("Ee"))return a.Ee;var b=new a;return a.Ee=b};var Dg=function(){var a={};this.g=function(b,c){return null!=a[b]?a[b]:c};this.s=function(){a[Bg.g]=!0}},Fg=function(a){return Eg().g(a.g,a.defaultValue)};var Gg=[];function Hg(){var a=$b("google_tag_data",{});a.ics||(a.ics={entries:{},set:Ig,update:Jg,addListener:Kg,notifyListeners:Lg,active:!1,usedDefault:!1});return a.ics}
function Ig(a,b,c,d,e,f){var g=Hg();g.active=!0;g.usedDefault=!0;if(void 0!=b){var h=g.entries,l=h[a]||{},m=l.region,p=c&&n(c)?c.toUpperCase():void 0;d=d.toUpperCase();e=e.toUpperCase();if(""===d||p===e||(p===d?m!==e:!p&&!m)){var q=!!(f&&0<f&&void 0===l.update),r={region:p,initial:"granted"===b,update:l.update,quiet:q};if(""!==d||!1!==l.initial)h[a]=r;q&&A.setTimeout(function(){h[a]===r&&r.quiet&&(r.quiet=!1,Mg(a),Lg(),Fa("TAGGING",2))},f)}}}
function Jg(a,b){var c=Hg();c.active=!0;if(void 0!=b){var d=Ng(a),e=c.entries,f=e[a]=e[a]||{};f.update="granted"===b;var g=Ng(a);f.quiet?(f.quiet=!1,Mg(a)):g!==d&&Mg(a)}}function Kg(a,b){Gg.push({se:a,ai:b})}function Mg(a){for(var b=0;b<Gg.length;++b){var c=Gg[b];Na(c.se)&&-1!==c.se.indexOf(a)&&(c.ug=!0)}}function Lg(a){for(var b=0;b<Gg.length;++b){var c=Gg[b];if(c.ug){c.ug=!1;try{c.ai({Rh:a})}catch(d){}}}}
var Ng=function(a){var b=Hg().entries[a]||{};return void 0!==b.update?b.update:b.initial},Og=function(a){return(Hg().entries[a]||{}).initial},Pg=function(a){return!(Hg().entries[a]||{}).quiet},Qg=function(){return Fg(Bg)?Hg().active:!1},Rg=function(){return Hg().usedDefault},Sg=function(a,b){Hg().addListener(a,b)},Tg=function(a){Hg().notifyListeners(a)},$g=function(a,b){function c(){for(var e=0;e<b.length;e++)if(!Pg(b[e]))return!0;return!1}if(c()){var d=!1;Sg(b,function(e){d||c()||(d=!0,a(e))})}else a({})},
ah=function(a,b){function c(){for(var f=[],g=0;g<d.length;g++){var h=d[g];!1===Ng(h)||e[h]||(f.push(h),e[h]=!0)}return f}var d=n(b)?[b]:b,e={};c().length!==d.length&&Sg(d,function(f){var g=c();0<g.length&&(f.se=g,a(f))})};function bh(a){for(var b=[],c=0;c<ch.length;c++){var d=a(ch[c]);b[c]=!0===d?"1":!1===d?"0":"-"}return b.join("")}
var ch=[N.D,N.H],dh=function(a){var b=a[N.Ad];b&&yg(40);var c=a[N.df];c&&yg(41);for(var d=Na(b)?b:[b],e={Kb:0};e.Kb<d.length;e={Kb:e.Kb},++e.Kb)Ua(a,function(f){return function(g,h){if(g!==N.Ad&&g!==N.df){var l=d[f.Kb];Hg().set(g,h,l,"DE","DE-HH",c)}}}(e))},eh=function(a,b){Ua(a,function(c,d){Hg().update(c,d)});Tg(b)},fh=function(a){var b=Ng(a);return void 0!=b?b:!0},gh=function(){return"G1"+bh(Ng)},hh=function(a,b){Sg(a,b)},ih=function(a,b){ah(a,b)},jh=function(a,b){$g(a,
b)};var lh=function(a){return kh?H.querySelectorAll(a):null},mh=function(a,b){if(!kh)return null;if(Element.prototype.closest)try{return a.closest(b)}catch(e){return null}var c=Element.prototype.matches||Element.prototype.webkitMatchesSelector||Element.prototype.mozMatchesSelector||Element.prototype.msMatchesSelector||Element.prototype.oMatchesSelector,d=a;if(!H.documentElement.contains(d))return null;do{try{if(c.call(d,b))return d}catch(e){break}d=d.parentElement||d.parentNode}while(null!==d&&1===d.nodeType);
return null},nh=!1;if(H.querySelectorAll)try{var oh=H.querySelectorAll(":root");oh&&1==oh.length&&oh[0]==H.documentElement&&(nh=!0)}catch(a){}var kh=nh;var ph=function(a){if(H.hidden)return!0;var b=a.getBoundingClientRect();if(b.top==b.bottom||b.left==b.right||!A.getComputedStyle)return!0;var c=A.getComputedStyle(a,null);if("hidden"===c.visibility)return!0;for(var d=a,e=c;d;){if("none"===e.display)return!0;var f=e.opacity,g=e.filter;if(g){var h=g.indexOf("opacity(");0<=h&&(g=g.substring(h+8,g.indexOf(")",h)),"%"==g.charAt(g.length-1)&&(g=g.substring(0,g.length-1)),f=Math.min(g,f))}if(void 0!==f&&0>=f)return!0;(d=d.parentElement)&&(e=A.getComputedStyle(d,
null))}return!1};
var qh=function(){var a=H.body,b=H.documentElement||a&&a.parentElement,c,d;if(H.compatMode&&"BackCompat"!==H.compatMode)c=b?b.clientHeight:0,d=b?b.clientWidth:0;else{var e=function(f,g){return f&&g?Math.min(f,g):Math.max(f,g)};yg(7);c=e(b?b.clientHeight:0,a?a.clientHeight:0);d=e(b?b.clientWidth:0,a?a.clientWidth:0)}return{width:d,height:c}},rh=function(a){var b=qh(),c=b.height,d=b.width,e=a.getBoundingClientRect(),f=e.bottom-e.top,g=e.right-e.left;return f&&g?(1-Math.min((Math.max(0-e.left,0)+Math.max(e.right-
d,0))/g,1))*(1-Math.min((Math.max(0-e.top,0)+Math.max(e.bottom-c,0))/f,1)):0};var sh=[],th=!(!A.IntersectionObserver||!A.IntersectionObserverEntry),uh=function(a,b,c){for(var d=new A.IntersectionObserver(a,{threshold:c}),e=0;e<b.length;e++)d.observe(b[e]);for(var f=0;f<sh.length;f++)if(!sh[f])return sh[f]=d,f;return sh.push(d)-1},vh=function(a,b,c){function d(h,l){var m={top:0,bottom:0,right:0,left:0,width:0,height:0},p={boundingClientRect:h.getBoundingClientRect(),
intersectionRatio:l,intersectionRect:m,isIntersecting:0<l,rootBounds:m,target:h,time:ab()};J(function(){return a(p)})}for(var e=[],f=[],g=0;g<b.length;g++)e.push(0),f.push(-1);c.sort(function(h,l){return h-l});return function(){for(var h=0;h<b.length;h++){var l=rh(b[h]);if(l>e[h])for(;f[h]<c.length-1&&l>=c[f[h]+1];)d(b[h],l),f[h]++;else if(l<e[h])for(;0<=f[h]&&l<=c[f[h]];)d(b[h],l),f[h]--;e[h]=l}}},wh=function(a,b,c){for(var d=0;d<c.length;d++)1<c[d]?c[d]=1:0>c[d]&&(c[d]=0);if(th){var e=!1;J(function(){e||
vh(a,b,c)()});return uh(function(f){e=!0;for(var g={Lb:0};g.Lb<f.length;g={Lb:g.Lb},g.Lb++)J(function(h){return function(){return a(f[h.Lb])}}(g))},b,c)}return A.setInterval(vh(a,b,c),1E3)},xh=function(a){th?0<=a&&a<sh.length&&sh[a]&&(sh[a].disconnect(),sh[a]=void 0):A.clearInterval(a)};var yh=/:[0-9]+$/,zh=function(a,b,c,d){for(var e=[],f=a.split("&"),g=0;g<f.length;g++){var h=f[g].split("=");if(decodeURIComponent(h[0]).replace(/\+/g," ")===b){var l=h.slice(1).join("=");if(!c)return d?l:decodeURIComponent(l).replace(/\+/g," ");e.push(d?l:decodeURIComponent(l).replace(/\+/g," "))}}return c?e:void 0},Ch=function(a,b,c,d,e){b&&(b=String(b).toLowerCase());if("protocol"===b||"port"===b)a.protocol=Ah(a.protocol)||Ah(A.location.protocol);"port"===b?a.port=String(Number(a.hostname?a.port:
A.location.port)||("http"==a.protocol?80:"https"==a.protocol?443:"")):"host"===b&&(a.hostname=(a.hostname||A.location.hostname).replace(yh,"").toLowerCase());return Bh(a,b,c,d,e)},Bh=function(a,b,c,d,e){var f,g=Ah(a.protocol);b&&(b=String(b).toLowerCase());switch(b){case "url_no_fragment":f=Dh(a);break;case "protocol":f=g;break;case "host":f=a.hostname.replace(yh,"").toLowerCase();if(c){var h=/^www\d*\./.exec(f);h&&h[0]&&(f=f.substr(h[0].length))}break;case "port":f=String(Number(a.port)||("http"==
g?80:"https"==g?443:""));break;case "path":a.pathname||a.hostname||Fa("TAGGING",1);f="/"==a.pathname.substr(0,1)?a.pathname:"/"+a.pathname;var l=f.split("/");0<=Oa(d||[],l[l.length-1])&&(l[l.length-1]="");f=l.join("/");break;case "query":f=a.search.replace("?","");e&&(f=zh(f,e,!1,void 0));break;case "extension":var m=a.pathname.split(".");f=1<m.length?m[m.length-1]:"";f=f.split("/")[0];break;case "fragment":f=a.hash.replace("#","");break;default:f=a&&a.href}return f},Ah=function(a){return a?a.replace(":",
"").toLowerCase():""},Dh=function(a){var b="";if(a&&a.href){var c=a.href.indexOf("#");b=0>c?a.href:a.href.substr(0,c)}return b},Eh=function(a){var b=H.createElement("a");a&&(b.href=a);var c=b.pathname;"/"!==c[0]&&(a||Fa("TAGGING",1),c="/"+c);var d=b.hostname.replace(yh,"");return{href:b.href,protocol:b.protocol,host:b.host,hostname:d,pathname:c,search:b.search,hash:b.hash,port:b.port}},Fh=function(a){function b(m){var p=m.split("=")[0];return 0>d.indexOf(p)?m:p+"=0"}function c(m){return m.split("&").map(b).filter(function(p){return void 0!=
p}).join("&")}var d="gclid dclid gbraid wbraid gclaw gcldc gclha gclgf gclgb _gl".split(" "),e=Eh(a),f=a.split(/[?#]/)[0],g=e.search,h=e.hash;"?"===g[0]&&(g=g.substring(1));"#"===h[0]&&(h=h.substring(1));g=c(g);h=c(h);""!==g&&(g="?"+g);""!==h&&(h="#"+h);var l=""+f+g+h;"/"===l[l.length-1]&&(l=l.substring(0,l.length-1));return l};var Gh={};var Hh=new RegExp(/[A-Z0-9._%+-]+@[A-Z0-9.-]+\.[A-Z]{2,}/i),Ih=new RegExp(/@(gmail|googlemail)\./i),Jh=new RegExp(/support|noreply/i),Kh="SCRIPT STYLE IMG SVG PATH BR".split(" "),Lh=["BR"],Mh={};
function Nh(a){var b;if(a===H.body)b="body";else{var c;if(a.id)c="#"+a.id;else{var d;if(a.parentElement){var e;a:{var f=a.parentElement;if(f){for(var g=0;g<f.childElementCount;g++)if(f.children[g]===a){e=g+1;break a}e=-1}else e=1}d=Nh(a.parentElement)+">:nth-child("+e+")"}else d="";c=d}b=c}return b}function Oh(a,b){if(1>=a.length)return a;var c=a.filter(b);return 0==c.length?a:c}
function Ph(a){if(0==a.length)return null;var b;b=Oh(a,function(c){return!Jh.test(c.ka)});b=Oh(b,function(c){return"INPUT"===c.element.tagName.toUpperCase()});b=Oh(b,function(c){return!ph(c.element)});return b[0]}
var Qh=function(a){var b=!a||!!a.ki,c=!a||!!a.li,d=b+"."+c;a&&a.ld&&a.ld.length&&(d+="."+a.ld.join("."));var e=Mh[d];if(e&&200>ab()-e.timestamp)return e.result;var f;var g=[],h=H.body;if(h){for(var l=h.querySelectorAll("*"),m=0;m<l.length&&1E4>m;m++){var p=l[m];if(!(0<=Kh.indexOf(p.tagName.toUpperCase()))){for(var q=!1,r=0;r<p.childElementCount&&1E4>r;r++)if(!(0<=Lh.indexOf(p.children[r].tagName.toUpperCase()))){q=!0;break}q||g.push(p)}}f={elements:g,status:1E4<l.length?"2":"1"}}else f={elements:g,
status:"4"};for(var t=f,u=t.elements,v=[],w=0;w<u.length;w++){var y=u[w],x=y.textContent;y.value&&(x=y.value);if(x){var z=x.match(Hh);if(z){var B=z[0],C;if(A.location){var E=Bh(A.location,"host",!0);C=0<=B.toLowerCase().indexOf(E)}else C=!1;C||v.push({element:y,ka:B})}}}var D;var I=a&&a.ld;if(I&&0!==I.length){for(var R=[],Q=0;Q<v.length;Q++){for(var T=!0,S=0;S<I.length;S++){var X=I[S];if(X&&mh(v[Q].element,X)){T=!1;break}}T&&R.push(v[Q])}D=R}else D=v;var K=Ph(D),U=[];if(K){var ba=K.element,O={ka:K.ka,
tagName:ba.tagName,type:1};b&&(O.querySelector=Nh(ba));c&&(O.isVisible=!ph(ba));U.push(O)}var ia={elements:U,status:10<v.length?"3":t.status};Mh[d]={timestamp:ab(),result:ia};return ia},Rh=function(a){return a.tagName+":"+a.isVisible+":"+a.ka.length+":"+Ih.test(a.ka)};var kf={},ji=null,ki=Math.random();kf.M="GTM-M6FZ495";kf.bd="6n0";kf.cj="";kf.Pg="ChEI8KzwhgYQiqjNtMCLlfC2ARIkAIM0oVCuAbQdhwPN+ooc7IHK43RYheYeRE8QxDQRr7hpETNNGgK/Mw\x3d\x3d";var li={__cl:!0,__ecl:!0,__ehl:!0,__evl:!0,__fal:!0,__fil:!0,__fsl:!0,__hl:!0,__jel:!0,__lcl:!0,__sdl:!0,__tl:!0,__ytl:!0},mi={__paused:!0,__tg:!0},ni;for(ni in li)li.hasOwnProperty(ni)&&(mi[ni]=!0);var oi="www.googletagmanager.com/gtm.js";
var pi=oi,qi=Xa(""),ri=null,si=null,ti="https://www.googletagmanager.com/a?id="+kf.M+"&cv=50",ui={},vi={},wi=function(){var a=ji.sequence||1;ji.sequence=a+1;return a};kf.Og="";var xi={},yi=new Sa,zi={},Ai={},Di={name:"dataLayer",set:function(a,b){L(kb(a,b),zi);Bi()},get:function(a){return Ci(a,2)},reset:function(){yi=new Sa;zi={};Bi()}},Ci=function(a,b){return 2!=b?yi.get(a):Ei(a)},Ei=function(a,b){var c=a.split(".");b=b||[];for(var d=zi,e=0;e<c.length;e++){if(null===d)return!1;if(void 0===d)break;d=d[c[e]];if(-1!==Oa(b,d))return}return d},Fi=function(a,b){Ai.hasOwnProperty(a)||(yi.set(a,b),L(kb(a,b),zi),Bi())},Gi=function(){for(var a=["gtm.allowlist","gtm.blocklist",
"gtm.whitelist","gtm.blacklist","tagTypeBlacklist"],b=0;b<a.length;b++){var c=a[b],d=Ci(c,1);if(Na(d)||Dc(d))d=L(d);Ai[c]=d}},Bi=function(a){Ua(Ai,function(b,c){yi.set(b,c);L(kb(b,void 0),zi);L(kb(b,c),zi);a&&delete Ai[b]})},Ii=function(a,b,c){xi[a]=xi[a]||{};xi[a][b]=Hi(b,c)},Hi=function(a,b){var c,d=1!==(void 0===b?2:b)?Ei(a):yi.get(a);"array"===Bc(d)||"object"===Bc(d)?c=L(d):c=d;return c},Ji=function(a,b){if(xi[a])return xi[a][b]},Ki=function(a,b){xi[a]&&delete xi[a][b]};var Ni={},Oi=function(a,b){if(A._gtmexpgrp&&A._gtmexpgrp.hasOwnProperty(a))return A._gtmexpgrp[a];void 0===Ni[a]&&(Ni[a]=Math.floor(Math.random()*b));return Ni[a]};function Pi(a,b,c){for(var d=[],e=b.split(";"),f=0;f<e.length;f++){var g=e[f].split("="),h=g[0].replace(/^\s*|\s*$/g,"");if(h&&h==a){var l=g.slice(1).join("=").replace(/^\s*|\s*$/g,"");l&&c&&(l=decodeURIComponent(l));d.push(l)}}return d};function Qi(a){return Fg(Cg)&&!a.navigator.cookieEnabled?!1:"null"!==a.origin};var Ti=function(a,b,c,d){return Ri(d)?Pi(a,String(b||Si()),c):[]},Wi=function(a,b,c,d,e){if(Ri(e)){var f=Ui(a,d,e);if(1===f.length)return f[0].id;if(0!==f.length){f=Vi(f,function(g){return g.kd},b);if(1===f.length)return f[0].id;f=Vi(f,function(g){return g.uc},c);return f[0]?f[0].id:void 0}}};function Xi(a,b,c,d){var e=Si(),f=window;Qi(f)&&(f.document.cookie=a);var g=Si();return e!=g||void 0!=c&&0<=Ti(b,g,!1,d).indexOf(c)}
var aj=function(a,b,c,d){function e(w,y,x){if(null==x)return delete h[y],w;h[y]=x;return w+"; "+y+"="+x}function f(w,y){if(null==y)return delete h[y],w;h[y]=!0;return w+"; "+y}if(!Ri(c.Sa))return 2;var g;void 0==b?g=a+"=deleted; expires="+(new Date(0)).toUTCString():(c.encode&&(b=encodeURIComponent(b)),b=Yi(b),g=a+"="+b);var h={};g=e(g,"path",c.path);var l;c.expires instanceof Date?l=c.expires.toUTCString():null!=c.expires&&(l=""+c.expires);g=e(g,"expires",l);g=e(g,"max-age",c.Ai);g=e(g,"samesite",
c.Oi);c.Qi&&(g=f(g,"secure"));var m=c.domain;if("auto"===m){for(var p=Zi(),q=void 0,r=!1,t=0;t<p.length;++t){var u="none"!==p[t]?p[t]:void 0,v=e(g,"domain",u);v=f(v,c.flags);try{d&&d(a,h)}catch(w){q=w;continue}r=!0;if(!$i(u,c.path)&&Xi(v,a,b,c.Sa))return 0}if(q&&!r)throw q;return 1}m&&"none"!==m&&(g=e(g,"domain",m));g=f(g,c.flags);d&&d(a,h);return $i(m,c.path)?1:Xi(g,a,b,c.Sa)?0:1},bj=function(a,b,c){null==c.path&&(c.path="/");c.domain||(c.domain="auto");return aj(a,b,c)};
function Vi(a,b,c){for(var d=[],e=[],f,g=0;g<a.length;g++){var h=a[g],l=b(h);l===c?d.push(h):void 0===f||l<f?(e=[h],f=l):l===f&&e.push(h)}return 0<d.length?d:e}function Ui(a,b,c){for(var d=[],e=Ti(a,void 0,void 0,c),f=0;f<e.length;f++){var g=e[f].split("."),h=g.shift();if(!b||-1!==b.indexOf(h)){var l=g.shift();l&&(l=l.split("-"),d.push({id:g.join("."),kd:1*l[0]||1,uc:1*l[1]||1}))}}return d}
var Yi=function(a){a&&1200<a.length&&(a=a.substring(0,1200));return a},cj=/^(www\.)?google(\.com?)?(\.[a-z]{2})?$/,dj=/(^|\.)doubleclick\.net$/i,$i=function(a,b){return dj.test(window.document.location.hostname)||"/"===b&&cj.test(a)},Si=function(){return Qi(window)?window.document.cookie:""},Zi=function(){var a=[],b=window.document.location.hostname.split(".");if(4===b.length){var c=b[b.length-1];if(parseInt(c,10).toString()===c)return["none"]}for(var d=b.length-2;0<=d;d--)a.push(b.slice(d).join("."));
var e=window.document.location.hostname;dj.test(e)||cj.test(e)||a.push("none");return a},Ri=function(a){if(!Fg(Bg)||!a||!Qg())return!0;if(!Pg(a))return!1;var b=Ng(a);return null==b?!0:!!b};var ej=function(){return[Math.round(2147483647*Math.random()),Math.round(ab()/1E3)].join(".")},hj=function(a,b,c,d,e){var f=fj(b);return Wi(a,f,gj(c),d,e)},ij=function(a,b,c,d){var e=""+fj(c),f=gj(d);1<f&&(e+="-"+f);return[b,e,a].join(".")},fj=function(a){if(!a)return 1;a=0===a.indexOf(".")?a.substr(1):a;return a.split(".").length},gj=function(a){if(!a||"/"===a)return 1;"/"!==a[0]&&(a="/"+a);"/"!==a[a.length-1]&&(a+="/");return a.split("/").length-1};function jj(a,b,c){var d,e=Number(null!=a.pb?a.pb:void 0);0!==e&&(d=new Date((b||ab())+1E3*(e||7776E3)));return{path:a.path,domain:a.domain,flags:a.flags,encode:!!c,expires:d}};var kj=["1"],lj={},pj=function(a){var b=mj(a.prefix);if(!lj[b]&&!nj(b,a.path,a.domain)){var c=ej();if(0===oj(b,c,a)){var d=$b("google_tag_data",{});d._gcl_au?Fa("GTM",57):d._gcl_au=c}nj(b,a.path,a.domain)}};function oj(a,b,c){var d=ij(b,"1",c.domain,c.path),e=jj(c);e.Sa="ad_storage";return bj(a,d,e)}function nj(a,b,c){var d=hj(a,b,c,kj,"ad_storage");d&&(lj[a]=d);return d}function mj(a){return(a||"_gcl")+"_au"};var qj=function(a){for(var b=[],c=H.cookie.split(";"),d=new RegExp("^\\s*"+(a||"_gac")+"_(UA-\\d+-\\d+)=\\s*(.+?)\\s*$"),e=0;e<c.length;e++){var f=c[e].match(d);f&&b.push({Ye:f[1],value:f[2],timestamp:Number(f[2].split(".")[1])||0})}b.sort(function(g,h){return h.timestamp-g.timestamp});return b};
function rj(a,b){var c=qj(a),d={};if(!c||!c.length)return d;for(var e=0;e<c.length;e++){var f=c[e].value.split(".");if(!("1"!==f[0]||b&&3>f.length||!b&&3!==f.length)&&Number(f[1])){d[c[e].Ye]||(d[c[e].Ye]=[]);var g={version:f[0],timestamp:1E3*Number(f[1]),ra:f[2]};b&&3<f.length&&(g.labels=f.slice(3));d[c[e].Ye].push(g)}}return d};function sj(){for(var a=tj,b={},c=0;c<a.length;++c)b[a[c]]=c;return b}function uj(){var a="ABCDEFGHIJKLMNOPQRSTUVWXYZ";a+=a.toLowerCase()+"0123456789-_";return a+"."}var tj,vj;
function wj(a){function b(l){for(;d<a.length;){var m=a.charAt(d++),p=vj[m];if(null!=p)return p;if(!/^[\s\xa0]*$/.test(m))throw Error("Unknown base64 encoding at char: "+m);}return l}tj=tj||uj();vj=vj||sj();for(var c="",d=0;;){var e=b(-1),f=b(0),g=b(64),h=b(64);if(64===h&&-1===e)return c;c+=String.fromCharCode(e<<2|f>>4);64!=g&&(c+=String.fromCharCode(f<<4&240|g>>2),64!=h&&(c+=String.fromCharCode(g<<6&192|h)))}};var xj;var Bj=function(){var a=yj,b=zj,c=Aj(),d=function(g){a(g.target||g.srcElement||{})},e=function(g){b(g.target||g.srcElement||{})};if(!c.init){hc(H,"mousedown",d);hc(H,"keyup",d);hc(H,"submit",e);var f=HTMLFormElement.prototype.submit;HTMLFormElement.prototype.submit=function(){b(this);f.call(this)};c.init=!0}},Cj=function(a,b,c,d,e){var f={callback:a,domains:b,fragment:2===c,placement:c,forms:d,sameHost:e};Aj().decorators.push(f)},Dj=function(a,b,c){for(var d=Aj().decorators,e={},f=0;f<d.length;++f){var g=
d[f],h;if(h=!c||g.forms)a:{var l=g.domains,m=a,p=!!g.sameHost;if(l&&(p||m!==H.location.hostname))for(var q=0;q<l.length;q++)if(l[q]instanceof RegExp){if(l[q].test(m)){h=!0;break a}}else if(0<=m.indexOf(l[q])||p&&0<=l[q].indexOf(m)){h=!0;break a}h=!1}if(h){var r=g.placement;void 0==r&&(r=g.fragment?2:1);r===b&&gb(e,g.callback())}}return e},Aj=function(){var a=$b("google_tag_data",{}),b=a.gl;b&&b.decorators||(b={decorators:[]},a.gl=b);return b};var Ej=/(.*?)\*(.*?)\*(.*)/,Fj=/^https?:\/\/([^\/]*?)\.?cdn\.ampproject\.org\/?(.*)/,Gj=/^(?:www\.|m\.|amp\.)+/,Hj=/([^?#]+)(\?[^#]*)?(#.*)?/;function Ij(a){return new RegExp("(.*?)(^|&)"+a+"=([^&]*)&?(.*)")}
var Kj=function(a){var b=[],c;for(c in a)if(a.hasOwnProperty(c)){var d=a[c];if(void 0!==d&&d===d&&null!==d&&"[object Object]"!==d.toString()){b.push(c);var e=b,f=e.push,g,h=String(d);tj=tj||uj();vj=vj||sj();for(var l=[],m=0;m<h.length;m+=3){var p=m+1<h.length,q=m+2<h.length,r=h.charCodeAt(m),t=p?h.charCodeAt(m+1):0,u=q?h.charCodeAt(m+2):0,v=r>>2,w=(r&3)<<4|t>>4,y=(t&15)<<2|u>>6,x=u&63;q||(x=64,p||(y=64));l.push(tj[v],tj[w],tj[y],tj[x])}g=l.join("");f.call(e,g)}}var z=b.join("*");return["1",Jj(z),
z].join("*")},Jj=function(a,b){var c=[window.navigator.userAgent,(new Date).getTimezoneOffset(),window.navigator.userLanguage||window.navigator.language,Math.floor((new Date).getTime()/60/1E3)-(void 0===b?0:b),a].join("*"),d;if(!(d=xj)){for(var e=Array(256),f=0;256>f;f++){for(var g=f,h=0;8>h;h++)g=g&1?g>>>1^3988292384:g>>>1;e[f]=g}d=e}xj=d;for(var l=4294967295,m=0;m<c.length;m++)l=l>>>8^xj[(l^c.charCodeAt(m))&255];return((l^-1)>>>0).toString(36)},Mj=function(){return function(a){var b=Eh(A.location.href),
c=b.search.replace("?",""),d=zh(c,"_gl",!1,!0)||"";a.query=Lj(d)||{};var e=Ch(b,"fragment").match(Ij("_gl"));a.fragment=Lj(e&&e[3]||"")||{}}},Nj=function(a){var b=Mj(),c=Aj();c.data||(c.data={query:{},fragment:{}},b(c.data));var d={},e=c.data;e&&(gb(d,e.query),a&&gb(d,e.fragment));return d},Lj=function(a){var b;b=void 0===b?3:b;try{if(a){var c;a:{for(var d=a,e=0;3>e;++e){var f=Ej.exec(d);if(f){c=f;break a}d=decodeURIComponent(d)}c=void 0}var g=c;if(g&&"1"===g[1]){var h=g[3],l;a:{for(var m=g[2],p=
0;p<b;++p)if(m===Jj(h,p)){l=!0;break a}l=!1}if(l){for(var q={},r=h?h.split("*"):[],t=0;t<r.length;t+=2)q[r[t]]=wj(r[t+1]);return q}}}}catch(u){}};function Oj(a,b,c,d){function e(p){var q=p,r=Ij(a).exec(q),t=q;if(r){var u=r[2],v=r[4];t=r[1];v&&(t=t+u+v)}p=t;var w=p.charAt(p.length-1);p&&"&"!==w&&(p+="&");return p+m}d=void 0===d?!1:d;var f=Hj.exec(c);if(!f)return"";var g=f[1],h=f[2]||"",l=f[3]||"",m=a+"="+b;d?l="#"+e(l.substring(1)):h="?"+e(h.substring(1));return""+g+h+l}
function Pj(a,b){var c="FORM"===(a.tagName||"").toUpperCase(),d=Dj(b,1,c),e=Dj(b,2,c),f=Dj(b,3,c);if(hb(d)){var g=Kj(d);c?Qj("_gl",g,a):Rj("_gl",g,a,!1)}if(!c&&hb(e)){var h=Kj(e);Rj("_gl",h,a,!0)}for(var l in f)if(f.hasOwnProperty(l))a:{var m=l,p=f[l],q=a;if(q.tagName){if("a"===q.tagName.toLowerCase()){Rj(m,p,q,void 0);break a}if("form"===q.tagName.toLowerCase()){Qj(m,p,q);break a}}"string"==typeof q&&Oj(m,p,q,void 0)}}
function Rj(a,b,c,d){if(c.href){var e=Oj(a,b,c.href,void 0===d?!1:d);Fb.test(e)&&(c.href=e)}}
function Qj(a,b,c){if(c&&c.action){var d=(c.method||"").toLowerCase();if("get"===d){for(var e=c.childNodes||[],f=!1,g=0;g<e.length;g++){var h=e[g];if(h.name===a){h.setAttribute("value",b);f=!0;break}}if(!f){var l=H.createElement("input");l.setAttribute("type","hidden");l.setAttribute("name",a);l.setAttribute("value",b);c.appendChild(l)}}else if("post"===d){var m=Oj(a,b,c.action);Fb.test(m)&&(c.action=m)}}}
var yj=function(a){try{var b;a:{for(var c=a,d=100;c&&0<d;){if(c.href&&c.nodeName.match(/^a(?:rea)?$/i)){b=c;break a}c=c.parentNode;d--}b=null}var e=b;if(e){var f=e.protocol;"http:"!==f&&"https:"!==f||Pj(e,e.hostname)}}catch(g){}},zj=function(a){try{if(a.action){var b=Ch(Eh(a.action),"host");Pj(a,b)}}catch(c){}},Sj=function(a,b,c,d){Bj();Cj(a,b,"fragment"===c?2:1,!!d,!1)},Tj=function(a,b){Bj();Cj(a,[Bh(A.location,"host",!0)],b,!0,!0)},Uj=function(){var a=H.location.hostname,b=Fj.exec(H.referrer);if(!b)return!1;
var c=b[2],d=b[1],e="";if(c){var f=c.split("/"),g=f[1];e="s"===g?decodeURIComponent(f[2]):decodeURIComponent(g)}else if(d){if(0===d.indexOf("xn--"))return!1;e=d.replace(/-/g,".").replace(/\.\./g,"-")}var h=a.replace(Gj,""),l=e.replace(Gj,""),m;if(!(m=h===l)){var p="."+l;m=h.substring(h.length-p.length,h.length)===p}return m},Vj=function(a,b){return!1===a?!1:a||b||Uj()};var Wj={};var Xj=/^\w+$/,dk=/^[\w-]+$/,ek={aw:"_aw",dc:"_dc",gf:"_gf",ha:"_ha",gp:"_gp",gb:"_gb"},fk=function(){if(!Fg(Bg)||!Qg())return!0;var a=Ng("ad_storage");return null==a?!0:!!a},gk=function(a,b){Pg("ad_storage")?fk()?a():ah(a,"ad_storage"):b?Fa("TAGGING",3):$g(function(){gk(a,!0)},["ad_storage"])},ik=function(a){return hk(a).map(function(b){return b.ra})},hk=function(a){var b=[];if(!Qi(A)||!H.cookie)return b;var c=Ti(a,H.cookie,void 0,"ad_storage");if(!c||0==c.length)return b;for(var d={},e=0;e<c.length;d=
{Dc:d.Dc},e++){var f=jk(c[e]);if(null!=f){var g=f,h=g.version;d.Dc=g.ra;var l=g.timestamp,m=g.labels,p=Qa(b,function(q){return function(r){return r.ra===q.Dc}}(d));p?(p.timestamp=Math.max(p.timestamp,l),p.labels=kk(p.labels,m||[])):b.push({version:h,ra:d.Dc,timestamp:l,labels:m})}}b.sort(function(q,r){return r.timestamp-q.timestamp});return lk(b)};function kk(a,b){for(var c={},d=[],e=0;e<a.length;e++)c[a[e]]=!0,d.push(a[e]);for(var f=0;f<b.length;f++)c[b[f]]||d.push(b[f]);return d}
function mk(a){return a&&"string"==typeof a&&a.match(Xj)?a:"_gcl"}
var ok=function(){var a=Eh(A.location.href),b=Ch(a,"query",!1,void 0,"gclid"),c=Ch(a,"query",!1,void 0,"gclsrc"),d=Ch(a,"query",!1,void 0,"wbraid"),e=Ch(a,"query",!1,void 0,"dclid");if(!b||!c||!d){var f=a.hash.replace("#","");b=b||zh(f,"gclid",!1,void 0);c=c||zh(f,"gclsrc",!1,void 0);d=d||zh(f,"wbraid",!1,void 0)}return nk(b,c,e,d)},nk=function(a,b,c,d){var e={},f=function(g,h){e[h]||(e[h]=[]);e[h].push(g)};e.gclid=a;e.gclsrc=b;e.dclid=c;void 0!==d&&dk.test(d)&&(e.gbraid=d,f(d,"gb"));if(void 0!==
a&&a.match(dk))switch(b){case void 0:f(a,"aw");break;case "aw.ds":f(a,"aw");f(a,"dc");break;case "ds":f(a,"dc");break;case "3p.ds":f(a,"dc");break;case "gf":f(a,"gf");break;case "ha":f(a,"ha")}c&&f(c,"dc");return e},pk=function(a,b){switch(a){case void 0:case "aw":return"aw"===b;case "aw.ds":return"aw"===b||"dc"===b;case "ds":case "3p.ds":return"dc"===b;case "gf":return"gf"===b;case "ha":return"ha"===b}return!1},rk=function(a){var b=ok();gk(function(){qk(b,a)})};
function qk(a,b,c,d){function e(p,q){var r=sk(p,f);r&&(bj(r,q,g),h=!0)}b=b||{};d=d||[];var f=mk(b.prefix);c=c||ab();var g=jj(b,c,!0);g.Sa="ad_storage";var h=!1,l=Math.round(c/1E3),m=function(p){var q=["GCL",l,p];0<d.length&&q.push(d.join("."));return q.join(".")};a.aw&&e("aw",m(a.aw[0]));a.dc&&e("dc",m(a.dc[0]));a.gf&&e("gf",m(a.gf[0]));a.ha&&e("ha",m(a.ha[0]));a.gp&&e("gp",m(a.gp[0]));(void 0==Wj.enable_gbraid_cookie_write?0:Wj.enable_gbraid_cookie_write)&&!h&&a.gb&&e("gb",m(a.gb[0]))}
var uk=function(a,b){var c=Nj(!0);gk(function(){for(var d=mk(b.prefix),e=0;e<a.length;++e){var f=a[e];if(void 0!==ek[f]){var g=sk(f,d),h=c[g];if(h){var l=Math.min(tk(h),ab()),m;b:{var p=l,q=g;if(Qi(A))for(var r=Ti(q,H.cookie,void 0,"ad_storage"),t=0;t<r.length;++t)if(tk(r[t])>p){m=!0;break b}m=!1}if(!m){var u=jj(b,l,!0);u.Sa="ad_storage";bj(g,h,u)}}}}qk(nk(c.gclid,c.gclsrc),b)})},sk=function(a,b){var c=ek[a];if(void 0!==c)return b+c},tk=function(a){return 0!==vk(a.split(".")).length?1E3*(Number(a.split(".")[1])||
0):0};function jk(a){var b=vk(a.split("."));return 0===b.length?null:{version:b[0],ra:b[2],timestamp:1E3*(Number(b[1])||0),labels:b.slice(3)}}function vk(a){return 3>a.length||"GCL"!==a[0]&&"1"!==a[0]||!/^\d+$/.test(a[1])||!dk.test(a[2])?[]:a}
var wk=function(a,b,c,d,e){if(Na(b)&&Qi(A)){var f=mk(e),g=function(){for(var h={},l=0;l<a.length;++l){var m=sk(a[l],f);if(m){var p=Ti(m,H.cookie,void 0,"ad_storage");p.length&&(h[m]=p.sort()[p.length-1])}}return h};gk(function(){Sj(g,b,c,d)})}},lk=function(a){return a.filter(function(b){return dk.test(b.ra)})},xk=function(a,b){if(Qi(A)){for(var c=mk(b.prefix),d={},e=0;e<a.length;e++)ek[a[e]]&&(d[a[e]]=ek[a[e]]);gk(function(){Ua(d,function(f,g){var h=Ti(c+g,H.cookie,void 0,"ad_storage");h.sort(function(t,
u){return tk(u)-tk(t)});if(h.length){var l=h[0],m=tk(l),p=0!==vk(l.split(".")).length?l.split(".").slice(3):[],q={},r;r=0!==vk(l.split(".")).length?l.split(".")[2]:void 0;q[f]=[r];qk(q,b,m,p)}})})}};function yk(a,b){for(var c=0;c<b.length;++c)if(a[b[c]])return!0;return!1}
var zk=function(a){function b(e,f,g){g&&(e[f]=g)}if(Qg()){var c=ok();if(yk(c,a)){var d={};b(d,"gclid",c.gclid);b(d,"dclid",c.dclid);b(d,"gclsrc",c.gclsrc);b(d,"wbraid",c.gbraid);Tj(function(){return d},3);Tj(function(){var e={};return e._up="1",e},1)}}};function Ak(a,b){var c=mk(b),d=sk(a,c);if(!d)return 0;for(var e=hk(d),f=0,g=0;g<e.length;g++)f=Math.max(f,e[g].timestamp);return f}
function Bk(a){var b=0,c;for(c in a)for(var d=a[c],e=0;e<d.length;e++)b=Math.max(b,Number(d[e].timestamp));return b};var Ck=/^\d+\.fls\.doubleclick\.net$/;function Dk(a,b){Pg(N.D)?fh(N.D)?a():ah(a,N.D):b?yg(42):jh(function(){Dk(a,!0)},[N.D])}function Ek(a){var b=Eh(A.location.href),c=Ch(b,"host",!1);if(c&&c.match(Ck)){var d=Ch(b,"path").split(a+"=");if(1<d.length)return d[1].split(";")[0].split("?")[0]}}
function Fk(a,b,c){if("aw"===a||"dc"===a||"gb"===a){var d=Ek("gcl"+a);if(d)return d.split(".")}var e=mk(b);if("_gcl"==e){c=void 0===c?!0:c;var f=!fh(N.D)&&c,g;g=ok()[a]||[];if(0<g.length)return f?["0"]:g}var h=sk(a,e);return h?ik(h):[]}function Gk(a){var b=[];Ua(a,function(c,d){d=lk(d);for(var e=[],f=0;f<d.length;f++)e.push(d[f].ra);e.length&&b.push(c+":"+e.join(","))});return b.join(";")}
var Hk=function(a){var b=Ek("gac");return b?!fh(N.D)&&a?"0":decodeURIComponent(b):Gk(fk()?rj():{})},Ik=function(a){var b=Ek("gacgb");return b?!fh(N.D)&&a?"0":decodeURIComponent(b):Gk(fk()?rj("_gac_gb",!0):{})},Jk=function(a,b,c){var d=ok(),e=[],f=d.gclid,g=d.dclid,h=d.gclsrc||"aw";!f||"aw.ds"!==h&&"aw"!==h&&"ds"!==h||c&&!pk(h,c)||e.push({ra:f,ze:h});!g||c&&"dc"!==c||e.push({ra:g,ze:"ds"});Dk(function(){pj(b);var l=lj[mj(b.prefix)],m=!1;if(l&&0<e.length)for(var p=ji.joined_auid=ji.joined_auid||{},q=0;q<e.length;q++){var r=e[q],t=r.ra,u=r.ze,v=(b.prefix||"_gcl")+"."+u+"."+t;if(!p[v]){var w="https://adservice.google.com/pagead/regclk";w="gb"===u?w+"?gbraid="+t+"&auid="+l:w+"?gclid="+t+"&auid="+l+"&gclsrc="+u;oc(w);m=p[v]=!0}}null==a&&(a=
m);if(a&&l){var y=mj(b.prefix),x=lj[y];x&&oj(y,x,b)}})},Kk=function(a){var b;if(Ek("gclaw")||Ek("gac")||0<(ok().aw||[]).length)b=!1;else{var c;if(0<(ok().gb||[]).length)c=!0;else{var d=Math.max(Ak("aw",a),Bk(fk()?rj():{}));c=Math.max(Ak("gb",a),Bk(fk()?rj("_gac_gb",!0):{}))>d}b=c}return b};var Lk=/[A-Z]+/,Mk=/\s/,Nk=function(a){if(n(a)&&(a=Za(a),!Mk.test(a))){var b=a.indexOf("-");if(!(0>b)){var c=a.substring(0,b);if(Lk.test(c)){for(var d=a.substring(b+1).split("/"),e=0;e<d.length;e++)if(!d[e])return;return{id:a,prefix:c,containerId:c+"-"+d[0],N:d}}}}},Pk=function(a){for(var b={},c=0;c<a.length;++c){var d=Nk(a[c]);d&&(b[d.id]=d)}Ok(b);var e=[];Ua(b,function(f,g){e.push(g)});return e};
function Ok(a){var b=[],c;for(c in a)if(a.hasOwnProperty(c)){var d=a[c];"AW"===d.prefix&&d.N[1]&&b.push(d.containerId)}for(var e=0;e<b.length;++e)delete a[b[e]]};var Qk=function(){var a=!1;return a};var Sk=function(a,b,c,d){return(2===Rk()||d||"http:"!=A.location.protocol?a:b)+c},Rk=function(){var a=ec(),b;if(1===a)a:{var c=pi;c=c.toLowerCase();for(var d="https://"+c,e="http://"+c,f=1,g=H.getElementsByTagName("script"),h=0;h<g.length&&100>h;h++){var l=g[h].src;if(l){l=l.toLowerCase();if(0===l.indexOf(e)){b=3;break a}1===f&&0===l.indexOf(d)&&(f=2)}}b=f}else b=a;return b};
var el=function(a){if(fh(N.D))return a;a=a.replace(/&url=([^&#]+)/,function(b,c){var d=Fh(decodeURIComponent(c));return"&url="+encodeURIComponent(d)});a=a.replace(/&ref=([^&#]+)/,function(b,c){var d=Fh(decodeURIComponent(c));return"&ref="+encodeURIComponent(d)});return a},fl=function(){var a;if(!(a=qi)){var b;if(!0===A._gtmdgs)b=!0;else{var c=Yb&&Yb.userAgent||"";b=0>c.indexOf("Safari")||/Chrome|Coast|Opera|Edg|Silk|Android/.test(c)||
11>((/Version\/([\d]+)/.exec(c)||[])[1]||"")?!1:!0}a=!b}if(a)return-1;var d=Wa("1");return Oi(1,100)<d?Oi(2,2):-1},gl=function(a){var b;if(!a||!a.length)return;for(var c=[],d=0;d<a.length;++d){var e=a[d];e&&e.estimated_delivery_date?c.push(""+e.estimated_delivery_date):c.push("")}b=c.join(",");return b};var hl=new RegExp(/^(.*\.)?(google|youtube|blogger|withgoogle)(\.com?)?(\.[a-z]{2})?\.?$/),il={cl:["ecl"],customPixels:["nonGooglePixels"],ecl:["cl"],ehl:["hl"],hl:["ehl"],html:["customScripts","customPixels","nonGooglePixels","nonGoogleScripts","nonGoogleIframes"],customScripts:["html","customPixels","nonGooglePixels","nonGoogleScripts","nonGoogleIframes"],nonGooglePixels:[],nonGoogleScripts:["nonGooglePixels"],nonGoogleIframes:["nonGooglePixels"]},jl={cl:["ecl"],customPixels:["customScripts","html"],
ecl:["cl"],ehl:["hl"],hl:["ehl"],html:["customScripts"],customScripts:["html"],nonGooglePixels:["customPixels","customScripts","html","nonGoogleScripts","nonGoogleIframes"],nonGoogleScripts:["customScripts","html"],nonGoogleIframes:["customScripts","html","nonGoogleScripts"]},kl="google customPixels customScripts html nonGooglePixels nonGoogleScripts nonGoogleIframes".split(" ");
var ll=function(){var a=!1;return a},nl=function(a){var b=Ci("gtm.allowlist")||Ci("gtm.whitelist");b&&yg(9);ll()&&(b="google gtagfl lcl zone oid op".split(" "));var c=b&&ib(Ya(b),il),d=Ci("gtm.blocklist")||
Ci("gtm.blacklist");d||(d=Ci("tagTypeBlacklist"))&&yg(3);d?yg(8):d=[];ml()&&(d=Ya(d),d.push("nonGooglePixels","nonGoogleScripts","sandboxedScripts"));0<=Oa(Ya(d),"google")&&yg(2);var e=d&&ib(Ya(d),jl),f={};return function(g){var h=g&&g[ne.kb];if(!h||"string"!=typeof h)return!0;h=h.replace(/^_*/,"");if(void 0!==f[h])return f[h];var l=vi[h]||[],m=a(h,l);if(b){var p;if(p=
m)a:{if(0>Oa(c,h))if(l&&0<l.length)for(var q=0;q<l.length;q++){if(0>Oa(c,l[q])){yg(11);p=!1;break a}}else{p=!1;break a}p=!0}m=p}var r=!1;if(d){var t=0<=Oa(e,h);if(t)r=t;else{var u=Ta(e,l||[]);u&&yg(10);r=u}}var v=!m||r;v||!(0<=Oa(l,"sandboxedScripts"))||c&&-1!==Oa(c,"sandboxedScripts")||(v=Ta(e,kl));return f[h]=v}},ml=function(){return hl.test(A.location&&A.location.hostname)};var ol={active:!0,isAllowed:function(){return!0}},pl=function(a){var b=ji.zones;return b?b.checkState(kf.M,a):ol},ql=function(a){var b=ji.zones;!b&&a&&(b=ji.zones=a());return b};var rl=function(){},sl=function(){};var tl=!1,ul=0,vl=[];function wl(a){if(!tl){var b=H.createEventObject,c="complete"==H.readyState,d="interactive"==H.readyState;if(!a||"readystatechange"!=a.type||c||!b&&d){tl=!0;for(var e=0;e<vl.length;e++)J(vl[e])}vl.push=function(){for(var f=0;f<arguments.length;f++)J(arguments[f]);return 0}}}function xl(){if(!tl&&140>ul){ul++;try{H.documentElement.doScroll("left"),wl()}catch(a){A.setTimeout(xl,50)}}}var yl=function(a){tl?a():vl.push(a)};var Al=function(a,b){this.g=!1;this.F=[];this.L={tags:[]};this.P=!1;this.s=this.B=0;zl(this,a,b)},Bl=function(a,b,c,d){if(mi.hasOwnProperty(b)||"__zone"===b)return-1;var e={};Dc(d)&&(e=L(d,e));e.id=c;e.status="timeout";return a.L.tags.push(e)-1},Cl=function(a,b,c,d){var e=a.L.tags[b];e&&(e.status=c,e.executionTime=d)},Dl=function(a){if(!a.g){for(var b=a.F,c=0;c<b.length;c++)b[c]();a.g=!0;a.F.length=0}},zl=function(a,b,c){Ja(b)&&a.bc(b);c&&A.setTimeout(function(){return Dl(a)},Number(c))};
Al.prototype.bc=function(a){var b=this,c=eb(function(){return J(function(){a(kf.M,b.L)})});this.g?c():this.F.push(c)};var El=function(a){a.B++;return eb(function(){a.s++;a.P&&a.s>=a.B&&Dl(a)})};var Fl=function(){function a(d){return!La(d)||0>d?0:d}if(!ji._li&&A.performance&&A.performance.timing){var b=A.performance.timing.navigationStart,c=La(Di.get("gtm.start"))?Di.get("gtm.start"):0;ji._li={cst:a(c-b),cbt:a(si-b)}}},Gl=function(a){A.performance&&A.performance.mark(kf.M+"_"+a+"_start")},Hl=function(a){if(A.performance){var b=kf.M+"_"+a+"_start",c=kf.M+"_"+a+"_duration";A.performance.measure(c,b);var d=A.performance.getEntriesByName(c)[0];A.performance.clearMarks(b);A.performance.clearMeasures(c);
var e=ji._p||{};void 0===e[a]&&(e[a]=d.duration,ji._p=e);return d.duration}},Il=function(){if(A.performance&&A.performance.now){var a=ji._p||{};a.PAGEVIEW=A.performance.now();ji._p=a}};var Jl={},Kl=function(){return A.GoogleAnalyticsObject&&A[A.GoogleAnalyticsObject]},Ll=!1;
var Ml=function(a){A.GoogleAnalyticsObject||(A.GoogleAnalyticsObject=a||"ga");var b=A.GoogleAnalyticsObject;if(A[b])A.hasOwnProperty(b)||yg(12);else{var c=function(){c.q=c.q||[];c.q.push(arguments)};c.l=Number($a());A[b]=c}Fl();return A[b]},Nl=function(a,b,c,d){b=String(b).replace(/\s+/g,"").split(",");var e=Kl();e(a+"require","linker");e(a+"linker:autoLink",b,c,d)},Ol=function(a){if(!Qg())return;var b=Kl();b(a+"require","linker");b(a+"linker:passthrough",
!0);};
var Ql=function(a){},Pl=function(){return A.GoogleAnalyticsObject||"ga"},Rl=function(a,b){return function(){var c=Kl(),d=c&&c.getByName&&c.getByName(a);if(d){var e=d.get("sendHitTask");d.set("sendHitTask",function(f){var g=f.get("hitPayload"),h=f.get("hitCallback"),l=0>g.indexOf("&tid="+b);l&&(f.set("hitPayload",g.replace(/&tid=UA-[0-9]+-[0-9]+/,"&tid="+
b),!0),f.set("hitCallback",void 0,!0));e(f);l&&(f.set("hitPayload",g,!0),f.set("hitCallback",h,!0),f.set("_x_19",void 0,!0),e(f))})}}};
var Yl=function(a){},bm=function(a){},cm=
function(){return"&tc="+Ne.filter(function(a){return a}).length},fm=function(){2022<=dm().length&&em()},hm=function(){gm||(gm=A.setTimeout(em,500))},em=function(){gm&&(A.clearTimeout(gm),gm=void 0);void 0===im||jm[im]&&!km&&!lm||(mm[im]||nm.oi()||0>=om--?(yg(1),mm[im]=!0):(nm.Ki(),gc(dm(!0)),jm[im]=!0,pm=qm=rm=lm=km=""))},dm=function(a){var b=im;if(void 0===b)return"";var c=Ha("GTM"),d=Ha("TAGGING");return[sm,jm[b]?"":"&es=1",tm[b],Yl(b),c?"&u="+c:"",d?"&ut="+d:"",cm(),km,lm,rm,qm,bm(a),pm,"&z=0"].join("")},
vm=function(){sm=um()},um=function(){return[ti,"&v=3&t=t","&pid="+Ra(),"&rv="+kf.bd].join("")},am=["L","S","Y"],Xl=["S","E"],wm={sampleRate:"0.005000",Jg:"",Ig:Number("5")},xm;if(!(xm=0<=H.location.search.indexOf("?gtm_latency=")||0<=H.location.search.indexOf("&gtm_latency="))){var ym=Math.random(),zm=wm.sampleRate;
xm=ym<zm}var Am=xm,sm=um(),jm={},km="",lm="",pm="",qm="",$l={},Wl={},Zl=!1,rm="",im=void 0,tm={},mm={},gm=void 0,Bm=2;0<wm.Ig&&(Bm=wm.Ig);var nm=function(a,b){for(var c=0,d=[],e=0;e<a;++e)d.push(0);return{oi:function(){return c<a?!1:ab()-d[c%a]<b},Ki:function(){var f=c++%a;d[f]=ab()}}}(Bm,1E3),om=1E3,Cm=function(a,b,c,d){if(Am&&!mm[a]&&b){a!==im&&(em(),
im=a);var e,f=String(b[ne.kb]||"").replace(/_/g,"");0===f.indexOf("cvt")&&(f="cvt");e=f;var g=c+e;km=km?km+"."+g:"&tr="+g;var h=b["function"];if(!h)throw Error("Error: No function name given for function call.");var l=(Pe[h]?"1":"2")+e;pm=pm?pm+"."+l:"&ti="+
l;hm();fm()}};var Fm=function(a,b,c){if(Am&&!mm[a]){a!==im&&(em(),im=a);var d=c+b;lm=lm?lm+"."+d:"&epr="+d;hm();fm()}},Gm=function(a,b,c){};function Hm(a,b,c,d){var e=Ne[a],f=Im(a,b,c,d);if(!f)return null;var g=Ve(e[ne.Tf],c,[]);if(g&&g.length){var h=g[0];f=Hm(h.index,{onSuccess:f,onFailure:1===h.lg?b.terminate:f,terminate:b.terminate},c,d)}return f}
function Im(a,b,c,d){function e(){if(f[ne.Bh])h();else{var w=We(f,c,[]);var y=w[ne.Ng];if(null!=y)for(var x=0;x<y.length;x++)if(!fh(y[x])){h();return}var z=Bl(c.Za,String(f[ne.kb]),Number(f[ne.Vf]),w[ne.Ch]),B=!1;w.vtp_gtmOnSuccess=function(){if(!B){B=!0;var D=ab()-E;Cm(c.id,Ne[a],"5",D);Cl(c.Za,z,"success",
D);g()}};w.vtp_gtmOnFailure=function(){if(!B){B=!0;var D=ab()-E;Cm(c.id,Ne[a],"6",D);Cl(c.Za,z,"failure",D);h()}};w.vtp_gtmTagId=f.tag_id;w.vtp_gtmEventId=c.id;Cm(c.id,f,"1");var C=function(){var D=ab()-E;Cm(c.id,f,"7",D);Cl(c.Za,z,"exception",D);B||(B=!0,h())};var E=ab();try{Ue(w,c)}catch(D){C(D)}}}var f=Ne[a],g=b.onSuccess,h=b.onFailure,l=b.terminate;if(c.Ge(f))return null;var m=Ve(f[ne.Wf],c,[]);if(m&&m.length){var p=m[0],q=Hm(p.index,{onSuccess:g,onFailure:h,terminate:l},c,d);if(!q)return null;g=q;h=2===p.lg?l:q}if(f[ne.Of]||f[ne.Eh]){var r=f[ne.Of]?Oe:
c.Vi,t=g,u=h;if(!r[a]){e=eb(e);var v=Jm(a,r,e);g=v.onSuccess;h=v.onFailure}return function(){r[a](t,u)}}return e}function Jm(a,b,c){var d=[],e=[];b[a]=Km(d,e,c);return{onSuccess:function(){b[a]=Lm;for(var f=0;f<d.length;f++)d[f]()},onFailure:function(){b[a]=Mm;for(var f=0;f<e.length;f++)e[f]()}}}function Km(a,b,c){return function(d,e){a.push(d);b.push(e);c()}}function Lm(a){a()}function Mm(a,b){b()};var Pm=function(a,b){for(var c=[],d=0;d<Ne.length;d++)if(a[d]){var e=Ne[d];if(e[ne.Ah])continue;var f=El(b.Za);try{var g=Hm(d,{onSuccess:f,onFailure:f,terminate:f},b,d);if(g){var h=c,l=h.push,m=d,p=e["function"];if(!p)throw"Error: No function name given for function call.";var q=Pe[p];l.call(h,{Eg:m,vg:q?q.priorityOverride||0:0,Gb:g})}else Nm(d,b),f()}catch(u){f()}}var r=b.Za;r.P=!0;r.s>=r.B&&Dl(r);c.sort(Om);for(var t=0;t<c.length;t++)c[t].Gb();
return 0<c.length};function Om(a,b){var c,d=b.vg,e=a.vg;c=d>e?1:d<e?-1:0;var f;if(0!==c)f=c;else{var g=a.Eg,h=b.Eg;f=g>h?1:g<h?-1:0}return f}function Nm(a,b){if(!Am)return;var c=function(d){var e=b.Ge(Ne[d])?"3":"4",f=Ve(Ne[d][ne.Tf],b,[]);f&&f.length&&c(f[0].index);Cm(b.id,Ne[d],e);var g=Ve(Ne[d][ne.Wf],b,[]);g&&g.length&&c(g[0].index)};c(a);}
var Qm=!1,Wm=function(a){var b=ab(),c=a["gtm.uniqueEventId"],d=a.event;if("gtm.js"===d){if(Qm)return!1;Qm=!0;}var g=pl(c),h=!1;if(!g.active){if("gtm.js"!==d)return!1;h=!0;g=pl(Number.MAX_SAFE_INTEGER)}
Am&&!mm[c]&&im!==c&&(em(),im=c,pm=km="",tm[c]="&e="+(0===d.indexOf("gtm.")?encodeURIComponent(d):"*")+"&eid="+c,hm());var l=a.eventCallback,m=a.eventTimeout,p=l;var q={id:c,name:d,Ge:nl(g.isAllowed),Vi:[],rg:function(){yg(6)},dg:Rm(c),Za:new Al(p,
m)};q.cg=Sm();Tm(c,q.Za);var r=ef(q);h&&(r=Um(r));var t=Pm(r,q);"gtm.js"!==d&&"gtm.sync"!==d||Ql(kf.M);
switch(d){case "gtm.init":t&&yg(20)}return Vm(r,t)};function Rm(a){return function(b){Am&&(Hc(b)||Gm(a,"input",b))}}function Tm(a,b){Ii(a,"event",1);Ii(a,"ecommerce",1);Ii(a,"gtm");Ii(a,"eventModel");}
function Sm(){var a={};a.event=Hi("event",1);a.ecommerce=Hi("ecommerce",1);a.gtm=Hi("gtm");a.eventModel=Hi("eventModel");return a}function Um(a){for(var b=[],c=0;c<a.length;c++)a[c]&&li[String(Ne[c][ne.kb])]&&(b[c]=!0);return b}function Vm(a,b){if(!b)return b;for(var c=0;c<a.length;c++)if(a[c]&&Ne[c]&&!mi[String(Ne[c][ne.kb])])return!0;return!1}function Xm(a,b){if(a){var c=""+a;0!==c.indexOf("http://")&&0!==c.indexOf("https://")&&(c="https://"+c);"/"===c[c.length-1]&&(c=c.substring(0,c.length-1));return Eh(""+c+b).href}}function Ym(a,b){return Zm()?Xm(a,b):void 0}function Zm(){var a=!1;return a};var $m=function(){this.eventModel={};this.targetConfig={};this.containerConfig={};this.remoteConfig={};this.globalConfig={};this.onSuccess=function(){};this.onFailure=function(){};this.setContainerTypeLoaded=function(){};this.getContainerTypeLoaded=function(){};this.eventId=void 0;this.isGtmEvent=!1},an=function(a){var b=new $m;b.eventModel=a;return b},bn=function(a,b){a.targetConfig=b;return a},cn=function(a,b){a.containerConfig=b;return a},dn=function(a,b){a.remoteConfig=b;return a},en=function(a,
b){a.globalConfig=b;return a},fn=function(a,b){a.onSuccess=b;return a},gn=function(a,b){a.setContainerTypeLoaded=b;return a},hn=function(a,b){a.getContainerTypeLoaded=b;return a},jn=function(a,b){a.onFailure=b;return a};
$m.prototype.getWithConfig=function(a){if(void 0!==this.eventModel[a])return this.eventModel[a];if(void 0!==this.targetConfig[a])return this.targetConfig[a];if(void 0!==this.containerConfig[a])return this.containerConfig[a];if(void 0!==this.remoteConfig[a])return this.remoteConfig[a];if(void 0!==this.globalConfig[a])return this.globalConfig[a]};
var kn=function(a){function b(e){Ua(e,function(f){c[f]=null})}var c={};b(a.eventModel);b(a.targetConfig);b(a.containerConfig);b(a.globalConfig);var d=[];Ua(c,function(e){d.push(e)});return d},ln=function(a,b){function c(f){Dc(f)&&Ua(f,function(g,h){e=!0;d[g]=h})}var d={},e=!1;c(a.globalConfig[b]);c(a.remoteConfig[b]);c(a.containerConfig[b]);c(a.targetConfig[b]);c(a.eventModel[b]);return e?d:void 0};var mn;if(3===kf.bd.length)mn="g";else{var nn="G";mn=nn}
var on={"":"n",UA:"u",AW:"a",DC:"d",G:"e",GF:"f",HA:"h",GTM:mn,OPT:"o"},pn=function(a){var b=kf.M.split("-"),c=b[0].toUpperCase(),d=on[c]||"i",e=a&&"GTM"===c?b[1]:"OPT"===c?b[1]:"",f;if(3===kf.bd.length){var g="w";f="2"+g}else f="";return f+d+kf.bd+e};var qn=function(a,b){a.addEventListener&&a.addEventListener.call(a,"message",b,!1)};var rn=function(){return Jb("iPhone")&&!Jb("iPod")&&!Jb("iPad")};Jb("Opera");Jb("Trident")||Jb("MSIE");Jb("Edge");!Jb("Gecko")||-1!=Gb.toLowerCase().indexOf("webkit")&&!Jb("Edge")||Jb("Trident")||Jb("MSIE")||Jb("Edge");-1!=Gb.toLowerCase().indexOf("webkit")&&!Jb("Edge")&&Jb("Mobile");Jb("Macintosh");Jb("Windows");Jb("Linux")||Jb("CrOS");var sn=pa.navigator||null;sn&&(sn.appVersion||"").indexOf("X11");Jb("Android");rn();Jb("iPad");Jb("iPod");rn()||Jb("iPad")||Jb("iPod");Gb.toLowerCase().indexOf("kaios");var tn=function(a,b){for(var c=a,d=0;50>d;++d){var e;try{e=!(!c.frames||!c.frames[b])}catch(h){e=!1}if(e)return c;var f;a:{try{var g=c.parent;if(g&&g!=c){f=g;break a}}catch(h){}f=null}if(!(c=f))break}return null},un=function(a){var b=H;b=void 0===b?window.document:b;if(!a||!b.head)return null;var c=document.createElement("meta");b.head.appendChild(c);c.httpEquiv="origin-trial";c.content=a;return c};var vn=function(){};var wn=function(a){void 0!==a.addtlConsent&&"string"!==typeof a.addtlConsent&&(a.addtlConsent=void 0);void 0!==a.gdprApplies&&"boolean"!==typeof a.gdprApplies&&(a.gdprApplies=void 0);return void 0!==a.tcString&&"string"!==typeof a.tcString||void 0!==a.listenerId&&"number"!==typeof a.listenerId?2:a.cmpStatus&&"error"!==a.cmpStatus?0:3},xn=function(a,b){this.s=a;this.g=null;this.F={};this.P=0;this.L=void 0===b?500:b;this.B=null};oa(xn,vn);
var zn=function(a){return"function"===typeof a.s.__tcfapi||null!=yn(a)};
xn.prototype.addEventListener=function(a){var b={},c=Vb(function(){return a(b)}),d=0;-1!==this.L&&(d=setTimeout(function(){b.tcString="tcunavailable";b.internalErrorState=1;c()},this.L));var e=function(f,g){clearTimeout(d);f?(b=f,b.internalErrorState=wn(b),g&&0===b.internalErrorState||(b.tcString="tcunavailable",g||(b.internalErrorState=3))):(b.tcString="tcunavailable",b.internalErrorState=3);a(b)};try{An(this,"addEventListener",e)}catch(f){b.tcString="tcunavailable",b.internalErrorState=3,d&&(clearTimeout(d),
d=0),c()}};xn.prototype.removeEventListener=function(a){a&&a.listenerId&&An(this,"removeEventListener",null,a.listenerId)};
var Cn=function(a,b,c){var d;d=void 0===d?"755":d;var e;a:{if(a.publisher&&a.publisher.restrictions){var f=a.publisher.restrictions[b];if(void 0!==f){e=f[void 0===d?"755":d];break a}}e=void 0}var g=e;if(0===g)return!1;var h=c;2===c?(h=0,2===g&&(h=1)):3===c&&(h=1,1===g&&(h=0));var l;if(0===h)if(a.purpose&&a.vendor){var m=Bn(a.vendor.consents,void 0===d?"755":d);l=m&&"1"===b&&a.purposeOneTreatment&&("DE"===a.publisherCC||Fg(Ag)&&"CH"===a.publisherCC)?!0:m&&Bn(a.purpose.consents,b)}else l=!0;else l=
1===h?a.purpose&&a.vendor?Bn(a.purpose.legitimateInterests,b)&&Bn(a.vendor.legitimateInterests,void 0===d?"755":d):!0:!0;return l},Bn=function(a,b){return!(!a||!a[b])},An=function(a,b,c,d){c||(c=function(){});if("function"===typeof a.s.__tcfapi){var e=a.s.__tcfapi;e(b,2,c,d)}else if(yn(a)){Dn(a);var f=++a.P;a.F[f]=c;if(a.g){var g={};a.g.postMessage((g.__tcfapiCall={command:b,version:2,callId:f,parameter:d},g),"*")}}else c({},!1)},yn=function(a){if(a.g)return a.g;a.g=tn(a.s,"__tcfapiLocator");return a.g},
Dn=function(a){a.B||(a.B=function(b){try{var c;c=("string"===typeof b.data?JSON.parse(b.data):b.data).__tcfapiReturn;a.F[c.callId](c.returnValue,c.success)}catch(d){}},qn(a.s,a.B))};var En=!0;En=!1;var Fn={1:0,3:0,4:0,7:3,9:3,10:3};function Gn(a,b){if(""===a)return b;var c=Number(a);return isNaN(c)?b:c}var Hn=Gn("",550),In=Gn("",500);function Jn(){var a=ji.tcf||{};return ji.tcf=a}
var Kn=function(a,b){this.B=a;this.g=b;this.s=ab();},Ln=function(a){},Mn=function(a){},Sn=function(){var a=Jn(),b=new xn(A,En?3E3:-1),c=new Kn(b,a);if((Nn()?!0===A.gtag_enable_tcf_support:!1!==A.gtag_enable_tcf_support)&&!a.active&&("function"===typeof A.__tcfapi||zn(b))){a.active=!0;a.wc={};On();var d=null;En?d=A.setTimeout(function(){Pn(a);Qn(a);d=null},In):a.tcString="tcunavailable";try{b.addEventListener(function(e){d&&(clearTimeout(d),d=null);if(0!==e.internalErrorState)Pn(a),Qn(a),Ln(c);
else{var f;a.gdprApplies=e.gdprApplies;if(!1===e.gdprApplies)f=Rn(),b.removeEventListener(e);else if("tcloaded"===e.eventStatus||"useractioncomplete"===e.eventStatus||"cmpuishown"===e.eventStatus){var g={},h;for(h in Fn)if(Fn.hasOwnProperty(h))if("1"===h){var l,m=e,p=!0;p=void 0===p?!1:p;var q;var r=m;!1===r.gdprApplies?q=!0:(void 0===r.internalErrorState&&(r.internalErrorState=wn(r)),q="error"===r.cmpStatus||0!==r.internalErrorState||"loaded"===r.cmpStatus&&("tcloaded"===r.eventStatus||"useractioncomplete"===
r.eventStatus)?!0:!1);l=q?!1===m.gdprApplies||"tcunavailable"===m.tcString||void 0===m.gdprApplies&&!p||"string"!==typeof m.tcString||!m.tcString.length?!0:Cn(m,"1",0):!1;g["1"]=l}else g[h]=Cn(e,h,Fn[h]);f=g}f&&(a.tcString=e.tcString||"tcempty",a.wc=f,Qn(a),Ln(c))}}),Mn(c)}catch(e){d&&(clearTimeout(d),d=null),Pn(a),Qn(a)}}};function Pn(a){a.type="e";a.tcString="tcunavailable";En&&(a.wc=Rn())}function On(){var a={},b=(a.ad_storage="denied",a.wait_for_update=Hn,a);dh(b)}
var Nn=function(){var a=!1;a=!0;return a};function Rn(){var a={},b;for(b in Fn)Fn.hasOwnProperty(b)&&(a[b]=!0);return a}function Qn(a){var b={},c=(b.ad_storage=a.wc["1"]?"granted":"denied",b);Tn();eh(c,0)}
var Un=function(){var a=Jn();if(a.active&&void 0!==a.loadTime)return Number(a.loadTime)},Tn=function(){var a=Jn();return a.active?a.tcString||"":""},Vn=function(){var a=Jn();return a.active&&void 0!==a.gdprApplies?a.gdprApplies?"1":"0":""},Wn=function(a){if(!Fn.hasOwnProperty(String(a)))return!0;var b=Jn();return b.active&&b.wc?!!b.wc[String(a)]:!0};var Xn=!1;Xn=!0;function Yn(a){var b=String(A.location).split(/[?#]/)[0],c=kf.Pg||A._CONSENT_MODE_SALT,d;if(a){var e;if(c){var f=b+a+c,g=1,h,l,m;if(f)for(g=0,l=f.length-1;0<=l;l--)m=f.charCodeAt(l),g=(g<<6&268435455)+m+(m<<14),h=g&266338304,g=0!=h?g^h>>21:g;e=String(g)}else e="0";d=e}else d="";return d}
function Zn(a){function b(u){var v;ji.reported_gclid||(ji.reported_gclid={});v=ji.reported_gclid;var w;w=Xn&&g&&(!Qg()||fh(N.D))?l+"."+(f.prefix||"_gcl")+(u?"gcu":"gcs"):l+(u?"gcu":"gcs");if(!v[w]){v[w]=!0;var y=[],x={},z=function(Q,T){T&&(y.push(Q+"="+encodeURIComponent(T)),x[Q]=!0)},B="https://www.google.com";if(Qg()){var C=fh(N.D);z("gcs",gh());u&&z("gcu","1");Rg()&&z("gcd","G1"+bh(Og));
ji.dedupe_gclid||(ji.dedupe_gclid=""+ej());z("rnd",ji.dedupe_gclid);if((!l||m&&"aw.ds"!==m)&&fh(N.D)){var E=ik("_gcl_aw");z("gclaw",E.join("."))}z("url",String(A.location).split(/[?#]/)[0]);z("dclid",$n(d,p));var D=!1;D=!0;C||!d&&!D||(B="https://pagead2.googlesyndication.com")}
z("gdpr_consent",Tn()),z("gdpr",Vn());"1"===Nj(!1)._up&&z("gtm_up","1");z("gclid",$n(d,l));z("gclsrc",m);if(!(x.gclid||x.dclid||x.gclaw)&&(z("gbraid",$n(d,q)),!x.gbraid&&Qg()&&fh(N.D))){var I=ik("_gcl_gb");z("gclgb",I.join("."))}z("gtm",pn(!e));Xn&&g&&fh(N.D)&&(pj(f||{}),z("auid",lj[mj(f.prefix)]||""));
a.ig&&z("did",a.ig);var R=B+"/pagead/landing?"+y.join("&");oc(R)}}var c=!!a.qe,d=!!a.sa,e=a.U,f=void 0===a.hd?{}:a.hd,g=void 0===a.pd?!0:a.pd,h=ok(),l=h.gclid||"",m=h.gclsrc,p=h.dclid||"",q=h.gbraid||"",r=!c&&((!l||m&&"aw.ds"!==m?!1:!0)||q),t=Qg();if(r||t)t?jh(function(){b();fh(N.D)||ih(function(u){return b(!0,u.Rh)},N.D)},[N.D]):b()}function $n(a,b){var c=a&&!fh(N.D);return b&&c?"0":b}var bp=function(){var a=!0;Wn(7)&&Wn(9)&&Wn(10)||(a=!1);var b=!0;b=!1;b&&!ap()&&(a=!1);return a},ap=function(){var a=!0;Wn(3)&&Wn(4)||(a=!1);return a};var Bp=!1;function Cp(){var a=ji;return a.gcq=a.gcq||new Dp}
var Ep=function(a,b,c){Cp().register(a,b,c)},Fp=function(a,b,c,d){Cp().push("event",[b,a],c,d)},Gp=function(a,b){Cp().push("config",[a],b)},Hp=function(a,b,c,d){Cp().push("get",[a,b],c,d)},Ip=function(a){return Cp().getRemoteConfig(a)},Jp={},Kp=function(){this.status=1;this.containerConfig={};this.targetConfig={};this.remoteConfig={};this.s={};this.B=null;this.g=!1},Lp=function(a,b,c,d,e){this.type=a;this.B=b;this.U=c||"";this.g=d;this.s=e},Dp=function(){this.s={};this.B={};this.g=[];this.F={AW:!1,
UA:!1};this.enableDeferrableCommandAfterConfig=Bp},Mp=function(a,b){var c=Nk(b);return a.s[c.containerId]=a.s[c.containerId]||new Kp},Np=function(a,b,c){if(b){var d=Nk(b);if(d&&1===Mp(a,b).status){Mp(a,b).status=2;var e={};Am&&(e.timeoutId=A.setTimeout(function(){yg(38);hm()},3E3));a.push("require",[e],d.containerId);Jp[d.containerId]=ab();if(Qk()){}else{var g="/gtag/js?id="+encodeURIComponent(d.containerId)+"&l=dataLayer&cx=c",h=("http:"!=A.location.protocol?"https:":"http:")+("//www.googletagmanager.com"+g),l=Ym(c,g)||h;dc(l)}}}},Op=function(a,b,c,d){if(d.U){var e=Mp(a,d.U),f=e.B;if(f){var g=L(c),h=L(e.targetConfig[d.U]),l=L(e.containerConfig),m=L(e.remoteConfig),p=L(a.B),q=Ci("gtm.uniqueEventId"),r=Nk(d.U).prefix,t=hn(gn(jn(fn(en(dn(cn(bn(an(g),
h),l),m),p),function(){Fm(q,r,"2");}),function(){Fm(q,r,"3");}),function(u,v){a.F[u]=v}),function(u){return a.F[u]});try{Fm(q,r,"1");f(d.U,b,d.B,t)}catch(u){Fm(q,r,"4");}}}};
Dp.prototype.register=function(a,b,c){var d=Mp(this,a);if(3!==d.status){d.B=b;d.status=3;if(c){L(d.remoteConfig,c);d.remoteConfig=c}var e=Nk(a),f=Jp[e.containerId];if(void 0!==f){var g=ji[e.containerId].bootstrap,h=e.prefix.toUpperCase();ji[e.containerId]._spx&&(h=h.toLowerCase());var l=Ci("gtm.uniqueEventId"),m=h,p=ab()-g;if(Am&&!mm[l]){l!==im&&(em(),im=l);var q=m+"."+Math.floor(g-
f)+"."+Math.floor(p);qm=qm?qm+","+q:"&cl="+q}delete Jp[e.containerId]}this.flush()}};Dp.prototype.push=function(a,b,c,d){var e=Math.floor(ab()/1E3);Np(this,c,b[0][N.na]||this.B[N.na]);Bp&&c&&Mp(this,c).g&&(d=!1);this.g.push(new Lp(a,e,c,b,d));d||this.flush()};Dp.prototype.insert=function(a,b,c){var d=Math.floor(ab()/1E3);0<this.g.length?this.g.splice(1,0,new Lp(a,d,c,b,!1)):this.g.push(new Lp(a,d,c,b,!1))};
Dp.prototype.flush=function(a){for(var b=this,c=[],d=!1,e={};this.g.length;){var f=this.g[0];if(f.s)Bp?!f.U||Mp(this,f.U).g?(f.s=!1,this.g.push(f)):c.push(f):(f.s=!1,this.g.push(f)),this.g.shift();else{switch(f.type){case "require":if(3!==Mp(this,f.U).status&&!a){Bp&&this.g.push.apply(this.g,c);return}Am&&A.clearTimeout(f.g[0].timeoutId);break;case "set":Ua(f.g[0],function(r,t){L(kb(r,t),b.B)});break;case "config":e.Ga={};Ua(f.g[0],function(r){return function(t,u){L(kb(t,u),r.Ga)}}(e));var g=!!e.Ga[N.Tc];
delete e.Ga[N.Tc];var h=Mp(this,f.U),l=Nk(f.U),m=l.containerId===l.id;g||(m?h.containerConfig={}:h.targetConfig[f.U]={});h.g&&g||Op(this,N.xa,e.Ga,f);h.g=!0;delete e.Ga[N.Xb];m?L(e.Ga,h.containerConfig):L(e.Ga,h.targetConfig[f.U]);Bp&&(d=!0);break;case "event":e.Cc={};Ua(f.g[0],function(r){return function(t,u){L(kb(t,u),r.Cc)}}(e));Op(this,f.g[1],e.Cc,f);break;case "get":var p={},q=(p[N.Wa]=f.g[0],p[N.Va]=f.g[1],p);Op(this,N.Ja,q,f)}this.g.shift();Pp(this,f)}e={Ga:e.Ga,Cc:e.Cc}}Bp&&(this.g.push.apply(this.g,
c),d&&this.flush())};var Pp=function(a,b){if("require"!==b.type)if(b.U)for(var c=a.getCommandListeners(b.U)[b.type]||[],d=0;d<c.length;d++)c[d]();else for(var e in a.s)if(a.s.hasOwnProperty(e)){var f=a.s[e];if(f&&f.s)for(var g=f.s[b.type]||[],h=0;h<g.length;h++)g[h]()}};Dp.prototype.getRemoteConfig=function(a){return Mp(this,a).remoteConfig};Dp.prototype.getCommandListeners=function(a){return Mp(this,a).s};function Qp(a,b){var c=this;};function Rp(a,b,c){};function Sp(a,b,c,d){};function Tp(a){};var Up=function(a,b,c){var d={event:b,"gtm.element":a,"gtm.elementClasses":pc(a,"className"),"gtm.elementId":a["for"]||kc(a,"id")||"","gtm.elementTarget":a.formTarget||pc(a,"target")||""};c&&(d["gtm.triggers"]=c.join(","));d["gtm.elementUrl"]=(a.attributes&&a.attributes.formaction?a.formAction:"")||a.action||pc(a,"href")||a.src||a.code||a.codebase||"";return d},Vp=function(a){ji.hasOwnProperty("autoEventsSettings")||(ji.autoEventsSettings={});var b=ji.autoEventsSettings;b.hasOwnProperty(a)||(b[a]=
{});return b[a]},Wp=function(a,b,c){Vp(a)[b]=c},Xp=function(a,b,c,d){var e=Vp(a),f=bb(e,b,d);e[b]=c(f)},Yp=function(a,b,c){var d=Vp(a);return bb(d,b,c)};var Zp=["input","select","textarea"],$p=["button","hidden","image","reset","submit"],aq=function(a){var b=a.tagName.toLowerCase();return!Qa(Zp,function(c){return c===b})||"input"===b&&Qa($p,function(c){return c===a.type.toLowerCase()})?!1:!0},bq=function(a){return a.form?a.form.tagName?a.form:H.getElementById(a.form):nc(a,["form"],100)},cq=function(a,b,c){if(!a.elements)return 0;for(var d=b.dataset[c],e=0,f=1;e<a.elements.length;e++){var g=a.elements[e];if(aq(g)){if(g.dataset[c]===d)return f;f++}}return 0};
function gq(a){};var hq={},iq=[],jq={},kq=0,lq=0;
function sq(a,b){}function tq(){};var uq={},vq=[];
var Cq=function(a,b){};

function Fq(a,b){};var Gq=/^\s*$/,Hq,Iq=!1;
function Tq(a,b){}function Uq(a,b,c){};var Vq=!!A.MutationObserver,Wq=void 0,Xq=function(a){if(!Wq){var b=function(){var c=H.body;if(c)if(Vq)(new MutationObserver(function(){for(var e=0;e<Wq.length;e++)J(Wq[e])})).observe(c,{childList:!0,subtree:!0});else{var d=!1;hc(c,"DOMNodeInserted",function(){d||(d=!0,J(function(){d=!1;for(var e=0;e<Wq.length;e++)J(Wq[e])}))})}};Wq=[];H.body?b():J(b)}Wq.push(a)};var Zq=["www.youtube.com","www.youtube-nocookie.com"],$q,ar=!1,br=0;
function lr(a,b){}function mr(a,b){return!0};function nr(a,b,c){};function or(a,b){var c;return c};function pr(a){};function qr(a){};var rr=!1,sr=[];function tr(){if(!rr){rr=!0;for(var a=0;a<sr.length;a++)J(sr[a])}}var ur=function(a){rr?J(a):sr.push(a)};function vr(a){M(G(this),["listener:!Fn"],arguments);gg(this,"process_dom_events","window","load");ur(Fc(a))};function wr(a){var b;return b};function xr(a,b){var c;var d=!1;var e=Ec(c,this.g,d);void 0===e&&void 0!==c&&yg(45);return e};function yr(a){var b;return b};function zr(a,b){var c=null,d=!1;M(G(this),["functionPath:!string","arrayPath:!string"],arguments);gg(this,"access_globals","readwrite",a);gg(this,"access_globals","readwrite",b);var e=[A,H],f=a.split("."),g=jb(f,e),h=f[f.length-1];if(void 0===g)throw Error("Path "+a+" does not exist.");var l=g[h];if(l&&!Ja(l))return null;
if(l)return Ec(l,this.g,d);var m;l=function(){if(!Ja(m.push))throw Error("Object at "+b+" in window is not an array.");m.push.call(m,arguments)};g[h]=l;var p=b.split("."),q=jb(p,e),r=p[p.length-1];if(void 0===q)throw Error("Path "+p+" does not exist.");m=q[r];void 0===m&&(m=[],q[r]=m);c=function(){l.apply(l,Array.prototype.slice.call(arguments,0))};return Ec(c,this.g,d)};function Ar(a){var b;var g=!1;return Ec(b,this.g,g)};var Br;function Cr(a){var b=!1;return b};var Dr=function(a){var b;return b};function Er(a,b){b=void 0===b?!0:b;var c;return c};function Fr(a){var b=null;return b};function Gr(a,b){var c;return c};function Hr(a,b){var c;return c};function Ir(a){var b="";return b};function Jr(a,b){var c;return c};function Kr(a){var b="";return b};function Lr(){gg(this,"get_user_agent");return A.navigator.userAgent};function Mr(a,b){};var Nr={};function Or(a,b,c,d,e,f){f?e[f]?(e[f][0].push(c),e[f][1].push(d)):(e[f]=[[c],[d]],dc(a,function(){for(var g=e[f][0],h=0;h<g.length;h++)J(g[h]);g.push=function(l){J(l);return 0}},function(){for(var g=e[f][1],h=0;h<g.length;h++)J(g[h]);e[f]=null},b)):dc(a,c,d,b)}
function Pr(a,b,c,d){M(G(this),["url:!string","onSuccess:?Fn","onFailure:?Fn","cacheToken:?string"],arguments);gg(this,"inject_script",a);var e=this.g;Or(a,void 0,function(){b&&b.s(e)},function(){c&&c.s(e)},Nr,d);}var Qr=Object.freeze({dl:1,id:1}),Rr={};
function Sr(a,b,c,d){};function Tr(a){var b=!0;return b};var Ur=function(){return!1},Vr={getItem:function(a){var b=null;return b},setItem:function(a,
b){return!1},removeItem:function(a){}};var Wr=["textContent","value","tagName","children","childElementCount"];
function Xr(a){var b;return b};function Yr(){};function Zr(a,b){var c;return c};function $r(a){var b=void 0;return b};function as(a,b){var c=!1;return c};function bs(){var a="";return a};function cs(){var a="";return a};var ds=["set","get","config","event"];
function es(a,b,c){};function fs(){};function gs(a,b,c,d){d=void 0===d?!1:d;};function hs(a,b,c){};function is(a,b,c,d){var e=this;d=void 0===d?!0:d;var f=!1;return f};function js(a){M(G(this),["consentSettings:!DustMap"],arguments);for(var b=a.lb(),c=b.length(),d=0;d<c;d++){var e=b.get(d);e!==N.Ad&&gg(this,"access_consent",e,"write")}dh(Fc(a))};function ks(a,b,c){M(G(this),["path:!string","value:?*","overrideExisting:?boolean"],arguments);gg(this,"access_globals","readwrite",a);var d=!1;var e=a.split("."),f=jb(e,[A,H]),g=e.pop();if(f&&(void 0===f[g]||c))return f[g]=Fc(b,this.g,d),!0;return!1};function ls(a,b,c){}
;var ms=function(a){for(var b=[],c=0,d=0;d<a.length;d++){var e=a.charCodeAt(d);128>e?b[c++]=e:(2048>e?b[c++]=e>>6|192:(55296==(e&64512)&&d+1<a.length&&56320==(a.charCodeAt(d+1)&64512)?(e=65536+((e&1023)<<10)+(a.charCodeAt(++d)&1023),b[c++]=e>>18|240,b[c++]=e>>12&63|128):b[c++]=e>>12|224,b[c++]=e>>6&63|128),b[c++]=e&63|128)}return b};function ns(a,b,c,d){var e=this;};function os(a,b,c){}
;var ps={},qs={};ps.getItem=function(a){var b=null;return b};
ps.setItem=function(a,b){};
ps.removeItem=function(a){};ps.clear=function(){};var rs=function(a){var b;return b};function ss(a){M(G(this),["consentSettings:!DustMap"],arguments);var b=Fc(a),c;for(c in b)b.hasOwnProperty(c)&&gg(this,"access_consent",c,"write");eh(b)};var ke=function(){var a=new tg;Qk()?(a.add("injectHiddenIframe",Ia),a.add("injectScript",Ia)):(a.add("injectHiddenIframe",Mr),a.add("injectScript",Pr));var b=hs;a.add("Math",$f());a.add("TestHelper",wg());a.add("addEventCallback",Tp);a.add("aliasInWindow",mr);a.add("assertApi",Wf);a.add("assertThat",Xf);a.add("callInWindow",
or);a.add("callLater",pr);a.add("copyFromDataLayer",xr);a.add("copyFromWindow",yr);a.add("createArgumentsQueue",zr);a.add("createQueue",Ar);a.add("decodeUri",ag);a.add("decodeUriComponent",bg);a.add("encodeUri",cg);a.add("encodeUriComponent",dg);a.add("fail",eg);a.add("fromBase64",Dr,!("atob"in A));a.add("generateRandom",fg);a.add("getContainerVersion",hg);a.add("getCookieValues",Er);a.add("getQueryParameters",Gr);a.add("getReferrerQueryParameters",Hr);a.add("getReferrerUrl",Ir);a.add("getTimestamp",
ig);a.add("getTimestampMillis",ig);a.add("getType",jg);a.add("getUrl",Kr);a.add("localStorage",Vr,!Ur());a.add("logToConsole",Yr);a.add("makeInteger",lg);a.add("makeNumber",mg);a.add("makeString",ng);a.add("makeTableMap",og);a.add("mock",qg);a.add("parseUrl",$r);a.add("queryPermission",as);a.add("readCharacterSet",bs);a.add("readTitle",cs);a.add("sendPixel",b);a.add("setCookie",is);a.add("setInWindow",ks);a.add("sha256",ns);a.add("templateStorage",ps);a.add("toBase64",rs,!("btoa"in A));a.add("JSON",
kg(function(d){var e=this.g.g;e&&e.log.call(this,"error",d)}));var c=!1;
c=!0;c&&a.add("setDefaultConsentState",js);a.add("updateConsentState",ss);
a.add("isConsentGranted",Tr);a.add("addConsentListener",Qp);
vg(a,"callOnWindowLoad",vr);
Qk()?vg(a,"internal.injectScript",Ia):vg(a,"internal.injectScript",Sr);
return function(d){var e;if(a.g.hasOwnProperty(d))e=a.get(d,this);else{var f;if(f=a.s.hasOwnProperty(d)){var g=!1,h=this.g.g;
if(h){var l=h.kc();if(l){0!==l.indexOf("__cvt_")&&(g=!0);}}f=g}if(f){var m=a.s.hasOwnProperty(d)?a.s[d]:void 0;m=ug(d,this)||m;e=m}else throw Error(d+" is not a valid API name.");}return e}};var ts=function(){var a=ji.consumeTestResult;if(a&&Ja(a))return!0;return!1},us=function(){var a={};return function(b,c,d){if(!ts())return;var e=a[b]||{testName:b,status:c,logMessages:[],elapsedTime:0};a[b]=e;switch(c){case 4:e.logMessages.push(d);break;case 3:d&&(e.error=d);break;case 5:e.returnValue=d}if(2===c||3===c){e.status=c;var f=ji.consumeTestResult;f&&f(e)}}};var ie,pf;
function vs(){var a=data.runtime||[],b=data.runtime_lines;ie=new ge;ws();Je=function(e,f,g){xs(f);var h=new tb;Ua(f,function(t,u){var v=Ec(u);void 0===v&&void 0!==u&&yg(44);h.set(t,v)});ie.g.g.L=bf();var l={Oh:qf(e),eventId:void 0!==g?g.id:void 0,bc:void 0!==g?function(t){return g.Za.bc(t)}:void 0,kc:function(){return e},log:function(){}};if(ts()){var m=us(),
p=void 0,q=void 0;l.wa={cc:{},Hb:function(t,u,v){1===u&&(p=t);7===u&&(q=v);m(t,u,v)},Je:pg()};l.log=function(t,u){if(p){var v=Array.prototype.slice.call(arguments,1);m(p,4,{level:t,source:q,message:v})}}}var r=je(l,[e,h]);ie.g.g.L=void 0;r instanceof ra&&"return"===r.g&&(r=r.s);return Fc(r)};le();for(var c=0;c<a.length;c++){var d=a[c];if(!Na(d)||3>d.length){if(0===d.length)continue;break}b&&b[c]&&b[c].length&&Ze(d,b[c]);ie.Gb(d)}}
function xs(a){var b=a.gtmOnSuccess,c=a.gtmOnFailure;Ja(b)&&(a.gtmOnSuccess=function(){J(b)});Ja(c)&&(a.gtmOnFailure=function(){J(c)})}function ws(){var a=ie;ji.SANDBOXED_JS_SEMAPHORE=ji.SANDBOXED_JS_SEMAPHORE||0;me(a,function(b,c,d){ji.SANDBOXED_JS_SEMAPHORE++;try{return b.apply(c,d)}finally{ji.SANDBOXED_JS_SEMAPHORE--}})}function ys(a){void 0!==a&&Ua(a,function(b,c){for(var d=0;d<c.length;d++){var e=c[d].replace(/^_*/,"");vi[e]=vi[e]||[];vi[e].push(b)}})};var zs="HA GF G UA AW DC".split(" "),As=!1,Bs={},Cs=!1;function Ds(a,b){var c={event:a};b&&(c.eventModel=L(b),b[N.Qd]&&(c.eventCallback=b[N.Qd]),b[N.Lc]&&(c.eventTimeout=b[N.Lc]));return c}function Es(a){a.hasOwnProperty("gtm.uniqueEventId")||Object.defineProperty(a,"gtm.uniqueEventId",{value:wi()});return a["gtm.uniqueEventId"]}
function Fs(){return As}
var Is={config:function(a){var b,c;c=Es(a);void 0===c&&(c=wi());return b},consent:function(a){function b(){Fs()&&L(a[2],{subcommand:a[1]})}if(3===a.length){yg(39);var c=wi(),d=a[1];"default"===d?(b(),dh(a[2])):"update"===d&&(b(),eh(a[2],c))}},event:function(a){var b=a[1];if(!(2>a.length)&&n(b)){var c;if(2<a.length){if(!Dc(a[2])&&
void 0!=a[2]||3<a.length)return;c=a[2]}var d=Ds(b,c),e=void 0;e=Es(a),d["gtm.uniqueEventId"]=e;void 0===e&&wi();return d}},get:function(a){},js:function(a){if(2==a.length&&a[1].getTime){Cs=!0;Fs();var b={event:"gtm.js","gtm.start":a[1].getTime()};b["gtm.uniqueEventId"]=Es(a);return b}},policy:function(a){if(3===a.length){var b=a[1],c=a[2],d=pf.s;d.g[b]?d.g[b].push(c):
d.g[b]=[c]}},set:function(a){var b;2==a.length&&Dc(a[1])?b=L(a[1]):3==a.length&&n(a[1])&&(b={},Dc(a[2])||Na(a[2])?b[a[1]]=L(a[2]):b[a[1]]=a[2]);if(b){b._clear=!0;return b}}},Js={policy:!0};var Ks=function(a,b){var c=a.hide;if(c&&void 0!==c[b]&&c.end){c[b]=!1;var d=!0,e;for(e in c)if(c.hasOwnProperty(e)&&!0===c[e]){d=!1;break}d&&(c.end(),c.end=null)}},Ms=function(a){var b=Ls(),c=b&&b.hide;c&&c.end&&(c[a]=!0)};var ct=function(a){if(bt(a))return a;this.g=a};ct.prototype.hi=function(){return this.g};var bt=function(a){return!a||"object"!==Bc(a)||Dc(a)?!1:"getUntrustedUpdateValue"in a};ct.prototype.getUntrustedUpdateValue=ct.prototype.hi;var dt=[],et=!1,ft=!1,gt=!1,ht=function(a){return A["dataLayer"].push(a)},it=function(a){var b=ji["dataLayer"],c=b?b.subscribers:1,d=0,e=a;return function(){++d===c&&(e(),e=null)}};
function jt(a){var b=a._clear;Ua(a,function(d,e){"_clear"!==d&&(b&&Fi(d,void 0),Fi(d,e))});ri||(ri=a["gtm.start"]);var c=a["gtm.uniqueEventId"];if(!a.event)return!1;c||(c=wi(),a["gtm.uniqueEventId"]=c,Fi("gtm.uniqueEventId",c));return Wm(a)}function kt(){var a=dt[0];if(null==a||"object"!==typeof a)return!1;if(a.event)return!0;if(Va(a)){var b=a[0];if("config"===b||"event"===b||"js"===b)return!0}return!1}
function lt(){for(var a=!1;!gt&&0<dt.length;){var b=!1;b=!0;if(b&&!ft&&kt()){var c={};dt.unshift((c.event=
"gtm.init",c));ft=!0}var d=!1;d=!0;if(d&&!et&&kt()){var e={};dt.unshift((e.event="gtm.init_consent",e));et=!0}gt=!0;delete zi.eventModel;Bi();var f=dt.shift();if(null!=f){var g=bt(f);
if(g){var h=f;f=bt(h)?h.getUntrustedUpdateValue():void 0;Gi()}try{if(Ja(f))try{f.call(Di)}catch(v){}else if(Na(f)){var l=f;if(n(l[0])){var m=l[0].split("."),p=m.pop(),q=l.slice(1),r=Ci(m.join("."),2);if(void 0!==r&&null!==r)try{r[p].apply(r,q)}catch(v){}}}else{if(Va(f)){a:{var t=f;if(t.length&&n(t[0])){var u=Is[t[0]];if(u&&(!g||!Js[t[0]])){f=u(t);break a}}f=void 0}if(!f){gt=!1;continue}}a=jt(f)||a}}finally{g&&Bi(!0)}}gt=!1}
return!a}function mt(){var b=lt();try{Ks(A["dataLayer"],kf.M)}catch(c){}return b}
var ot=function(){var a=$b("dataLayer",[]),b=$b("google_tag_manager",{});b=b["dataLayer"]=b["dataLayer"]||{};yl(function(){b.gtmDom||(b.gtmDom=!0,a.push({event:"gtm.dom"}))});ur(function(){b.gtmLoad||(b.gtmLoad=!0,a.push({event:"gtm.load"}))});b.subscribers=(b.subscribers||0)+1;var c=a.push;a.push=function(){var e;if(0<ji.SANDBOXED_JS_SEMAPHORE){e=[];for(var f=0;f<arguments.length;f++)e[f]=new ct(arguments[f])}else e=[].slice.call(arguments,0);var g=c.apply(a,e);dt.push.apply(dt,e);if(300<
this.length)for(yg(4);300<this.length;)this.shift();var h="boolean"!==typeof g||g;return lt()&&h};var d=a.slice(0);dt.push.apply(dt,d);if(nt()){J(mt)}},nt=function(){var a=!0;return a};var pt={};pt.Xc=new String("undefined");
var qt=function(a){this.g=function(b){for(var c=[],d=0;d<a.length;d++)c.push(a[d]===pt.Xc?b:a[d]);return c.join("")}};qt.prototype.toString=function(){return this.g("undefined")};qt.prototype.valueOf=qt.prototype.toString;pt.Gh=qt;pt.je={};pt.Wh=function(a){return new qt(a)};var rt={};pt.Li=function(a,b){var c=wi();rt[c]=[a,b];return c};pt.gg=function(a){var b=a?0:1;return function(c){var d=rt[c];if(d&&"function"===typeof d[b])d[b]();rt[c]=void 0}};pt.ni=function(a){for(var b=!1,c=!1,d=2;d<a.length;d++)b=
b||8===a[d],c=c||16===a[d];return b&&c};pt.Hi=function(a){if(a===pt.Xc)return a;var b=wi();pt.je[b]=a;return'google_tag_manager["'+kf.M+'"].macro('+b+")"};pt.Bi=function(a,b,c){a instanceof pt.Gh&&(a=a.g(pt.Li(b,c)),b=Ia);return{ii:a,onSuccess:b}};var Ct=A.clearTimeout,Dt=A.setTimeout,P=function(a,b,c){if(Qk()){b&&J(b)}else return dc(a,b,c)},Et=function(){return new Date},Ft=function(){return A.location.href},Gt=function(a){return Ch(Eh(a),"fragment")},Ht=function(a){return Dh(Eh(a))},It=function(a,b){return Ci(a,b||2)},Jt=function(a,b,c){var d;b?(a.eventCallback=b,c&&(a.eventTimeout=c),d=ht(a)):d=ht(a);return d},Kt=function(a,b){A[a]=b},V=function(a,b,c){b&&
(void 0===A[a]||c&&!A[a])&&(A[a]=b);return A[a]},Lt=function(a,b,c){return Ti(a,b,void 0===c?!0:!!c)},Mt=function(a,b,c){return 0===bj(a,b,c)},Nt=function(a,b){if(Qk()){b&&J(b)}else fc(a,b)},Ot=function(a){return!!Yp(a,"init",!1)},Pt=function(a){Wp(a,"init",!0)},Qt=function(a){var b=pi+"?id="+encodeURIComponent(a)+"&l=dataLayer";P(Sk("https://","http://",b))},Rt=function(a,b,c){Am&&(Hc(a)||Gm(c,b,a))};var St=pt.Bi;function ou(a,b){a=String(a);b=String(b);var c=a.length-b.length;return 0<=c&&a.indexOf(b,c)==c}var pu=new Sa;function qu(a,b,c){var d=c?"i":void 0;try{var e=String(b)+d,f=pu.get(e);f||(f=new RegExp(b,d),pu.set(e,f));return f.test(a)}catch(g){return!1}}
function ru(a,b){function c(g){var h=Eh(g),l=Ch(h,"protocol"),m=Ch(h,"host",!0),p=Ch(h,"port"),q=Ch(h,"path").toLowerCase().replace(/\/$/,"");if(void 0===l||"http"==l&&"80"==p||"https"==l&&"443"==p)l="web",p="default";return[l,m,p,q]}for(var d=c(String(a)),e=c(String(b)),f=0;f<d.length;f++)if(d[f]!==e[f])return!1;return!0}
function su(a){return tu(a)?1:0}
function tu(a){var b=a.arg0,c=a.arg1;if(a.any_of&&Na(c)){for(var d=0;d<c.length;d++){var e=L(a,{});L({arg1:c[d],any_of:void 0},e);if(su(e))return!0}return!1}switch(a["function"]){case "_cn":return 0<=String(b).indexOf(String(c));case "_css":var f;a:{if(b){var g=["matches","webkitMatchesSelector","mozMatchesSelector","msMatchesSelector","oMatchesSelector"];try{for(var h=0;h<g.length;h++)if(b[g[h]]){f=b[g[h]](c);break a}}catch(m){}}f=!1}return f;case "_ew":return ou(b,c);case "_eq":return String(b)==
String(c);case "_ge":return Number(b)>=Number(c);case "_gt":return Number(b)>Number(c);case "_lc":var l;l=String(b).split(",");return 0<=Oa(l,String(c));case "_le":return Number(b)<=Number(c);case "_lt":return Number(b)<Number(c);case "_re":return qu(b,c,a.ignore_case);case "_sw":return 0==String(b).indexOf(String(c));case "_um":return ru(b,c)}return!1};var yu=encodeURI,W=encodeURIComponent,zu=gc;var Au=function(a,b){if(!a)return!1;var c=Ch(Eh(a),"host");if(!c)return!1;for(var d=0;b&&d<b.length;d++){var e=b[d]&&b[d].toLowerCase();if(e){var f=c.length-e.length;0<f&&"."!=e.charAt(0)&&(f--,e="."+e);if(0<=f&&c.indexOf(e,f)==f)return!0}}return!1};
var Bu=function(a,b,c){for(var d={},e=!1,f=0;a&&f<a.length;f++)a[f]&&a[f].hasOwnProperty(b)&&a[f].hasOwnProperty(c)&&(d[a[f][b]]=a[f][c],e=!0);return e?d:null};function iw(){return A.gaGlobal=A.gaGlobal||{}}var jw=function(){var a=iw();a.hid=a.hid||Ra();return a.hid},kw=function(a,b){var c=iw();if(void 0==c.vid||b&&!c.from_cookie)c.vid=a,c.from_cookie=b};var Iw=function(){if(Ja(A.__uspapi)){var a="";try{A.__uspapi("getUSPData",1,function(b,c){if(c&&b){var d=b.uspString;d&&/^[\da-zA-Z-]{1,20}$/.test(d)&&(a=d)}})}catch(b){}return a}};var cx=window,dx=document,ex=function(a){var b=cx._gaUserPrefs;if(b&&b.ioo&&b.ioo()||a&&!0===cx["ga-disable-"+a])return!0;try{var c=cx.external;if(c&&c._gaUserPrefs&&"oo"==c._gaUserPrefs)return!0}catch(f){}for(var d=Pi("AMP_TOKEN",String(dx.cookie),!0),e=0;e<d.length;e++)if("$OPT_OUT"==d[e])return!0;return dx.getElementById("__gaOptOutExtension")?!0:!1};var fx={};function ix(a){delete a.eventModel[N.Xb];kx(a.eventModel)}var kx=function(a){Ua(a,function(c){"_"===c.charAt(0)&&delete a[c]});var b=a[N.Ma]||{};Ua(b,function(c){"_"===c.charAt(0)&&delete b[c]})};var nx=function(a,b,c){Fp(b,c,a)},ox=function(a,b,c){Fp(b,c,a,!0)},vx=function(a,b){};
function px(a,b){}var Y={h:{}};

Y.h.d=["google"],function(){(function(a){Y.__d=a;Y.__d.m="d";Y.__d.o=!0;Y.__d.priorityOverride=0})(function(a){var b=null,c=null,d=a.vtp_attributeName;if("CSS"==a.vtp_selectorType)try{var e=lh(a.vtp_elementSelector);e&&0<e.length&&(b=e[0])}catch(f){b=null}else b=H.getElementById(a.vtp_elementId);b&&(d?c=kc(b,d):c=lc(b));return Za(String(b&&c))})}();
Y.h.e=["google"],function(){(function(a){Y.__e=a;Y.__e.m="e";Y.__e.o=!0;Y.__e.priorityOverride=0})(function(a){var b=String(Ji(a.vtp_gtmEventId,"event"));a.vtp_gtmCachedValues&&(b=String(a.vtp_gtmCachedValues.event));return b})}();
Y.h.f=["google"],function(){(function(a){Y.__f=a;Y.__f.m="f";Y.__f.o=!0;Y.__f.priorityOverride=0})(function(a){var b=It("gtm.referrer",1)||H.referrer;return b?a.vtp_component&&"URL"!=a.vtp_component?Ch(Eh(String(b)),a.vtp_component,a.vtp_stripWww,a.vtp_defaultPages,a.vtp_queryKey):Ht(String(b)):String(b)})}();
Y.h.cl=["google"],function(){function a(b){var c=b.target;if(c){var d=Up(c,"gtm.click");Jt(d)}}(function(b){Y.__cl=b;Y.__cl.m="cl";Y.__cl.o=!0;Y.__cl.priorityOverride=0})(function(b){if(!Ot("cl")){var c=V("document");hc(c,"click",a,!0);Pt("cl")}J(b.vtp_gtmOnSuccess)})}();
Y.h.j=["google"],function(){(function(a){Y.__j=a;Y.__j.m="j";Y.__j.o=!0;Y.__j.priorityOverride=0})(function(a){for(var b=String(a.vtp_name).split("."),c=V(b.shift()),d=0;d<b.length;d++)c=c&&c[b[d]];Rt(c,"j",a.vtp_gtmEventId);return c})}();

Y.h.access_globals=["google"],function(){function a(b,c,d){var e={key:d,read:!1,write:!1,execute:!1};switch(c){case "read":e.read=!0;break;case "write":e.write=!0;break;case "readwrite":e.read=e.write=!0;break;case "execute":e.execute=!0;break;default:throw Error("Invalid "+b+" request "+c);}return e}(function(b){Y.__access_globals=b;Y.__access_globals.m="access_globals";Y.__access_globals.o=!0;Y.__access_globals.priorityOverride=0})(function(b){for(var c=b.vtp_keys||[],d=b.vtp_createPermissionError,
e=[],f=[],g=[],h=0;h<c.length;h++){var l=c[h],m=l.key;l.read&&e.push(m);l.write&&f.push(m);l.execute&&g.push(m)}return{assert:function(p,q,r){if(!n(r))throw d(p,{},"Key must be a string.");if("read"===q){if(-1<Oa(e,r))return}else if("write"===q){if(-1<Oa(f,r))return}else if("readwrite"===q){if(-1<Oa(f,r)&&-1<Oa(e,r))return}else if("execute"===q){if(-1<Oa(g,r))return}else throw d(p,{},"Operation must be either 'read', 'write', or 'execute', was "+q);throw d(p,{},"Prohibited "+q+" on global variable: "+
r+".");},T:a}})}();
Y.h.u=["google"],function(){var a=function(b){return{toString:function(){return b}}};(function(b){Y.__u=b;Y.__u.m="u";Y.__u.o=!0;Y.__u.priorityOverride=0})(function(b){var c;c=(c=b.vtp_customUrlSource?b.vtp_customUrlSource:It("gtm.url",1))||Ft();var d=b[a("vtp_component")];if(!d||"URL"==d)return Ht(String(c));var e=Eh(String(c)),f;if("QUERY"===d)a:{var g=b[a("vtp_multiQueryKeys").toString()],h=b[a("vtp_queryKey").toString()]||"",l=b[a("vtp_ignoreEmptyQueryParam").toString()],m;g?Na(h)?m=h:m=String(h).replace(/\s+/g,
"").split(","):m=[String(h)];for(var p=0;p<m.length;p++){var q=Ch(e,"QUERY",void 0,void 0,m[p]);if(void 0!=q&&(!l||""!==q)){f=q;break a}}f=void 0}else f=Ch(e,d,"HOST"==d?b[a("vtp_stripWww")]:void 0,"PATH"==d?b[a("vtp_defaultPages")]:void 0,void 0);return f})}();
Y.h.v=["google"],function(){(function(a){Y.__v=a;Y.__v.m="v";Y.__v.o=!0;Y.__v.priorityOverride=0})(function(a){var b=a.vtp_name;if(!b||!b.replace)return!1;var c=It(b.replace(/\\\./g,"."),a.vtp_dataLayerVersion||1),d=void 0!==c?c:a.vtp_defaultValue;Rt(d,"v",a.vtp_gtmEventId);return d})}();
Y.h.ua=["google"],function(){function a(q){return fh(q)}function b(q,r,t){var u=!1;if(Qg()&&!u&&!e[q]){var v=!fh(N.H),w=function(){var y=Kl(),x="gtm"+wi(),z=m(r);z["&gtm"]=pn(!0);var B={name:x};l(z,B,!0);var C=void 0,E=B._useUp;y(function(){var D=y.getByName(t);D&&(C=D.get("clientId"))});
y("create",q,B);v&&fh(N.H)&&(v=!1,y(function(){var D=y.getByName(x);!D||D.get("clientId")===C&&E||(z["&gcs"]=gh(),z["&gcu"]="1",D.set(z),D.send("pageview"))}));y(function(){y.remove(x)})};ah(w,N.H);ah(w,N.D);e[q]=!0}}var c,d={},e={},f={name:!0,clientId:!0,sampleRate:!0,siteSpeedSampleRate:!0,alwaysSendReferrer:!0,allowAnchor:!0,allowLinker:!0,cookieName:!0,cookieDomain:!0,cookieExpires:!0,
cookiePath:!0,cookieUpdate:!0,cookieFlags:!0,legacyCookieDomain:!0,legacyHistoryImport:!0,storage:!0,useAmpClientId:!0,storeGac:!0,_cd2l:!0,_useUp:!0,_cs:!0},g={allowAnchor:!0,allowLinker:!0,alwaysSendReferrer:!0,anonymizeIp:!0,cookieUpdate:!0,exFatal:!0,forceSSL:!0,javaEnabled:!0,legacyHistoryImport:!0,nonInteraction:!0,useAmpClientId:!0,useBeacon:!0,storeGac:!0,allowAdFeatures:!0,allowAdPersonalizationSignals:!0,_cd2l:!0},h={urlPassthrough:!0},l=function(q,r,t){var u=0;if(q)for(var v in q)if(!h[v]&&
q.hasOwnProperty(v)&&(t&&f[v]||!t&&void 0===f[v])){var w=g[v]?Xa(q[v]):q[v];"anonymizeIp"!=v||w||(w=void 0);r[v]=w;u++}return u},m=function(q){var r={};q.vtp_gaSettings&&L(Bu(q.vtp_gaSettings.vtp_fieldsToSet,"fieldName","value"),r);L(Bu(q.vtp_fieldsToSet,"fieldName","value"),r);fh(N.H)||(r.storage="none");fh(N.D)||(r.allowAdFeatures=!1,r.storeGac=!1);bp()||(r.allowAdFeatures=!1);ap()||(r.allowAdPersonalizationSignals=!1);q.vtp_transportUrl&&(r._x_19=q.vtp_transportUrl);if(Xa(r.urlPassthrough)){r._useUp=!0;var t=!1;Qg()&&!t&&(r._cs=a)}return r},p=function(q){function r(Ca,ka){void 0!==ka&&D("set",Ca,ka)}var t={},u={},v={},w={};if(q.vtp_gaSettings){var y=
q.vtp_gaSettings;L(Bu(y.vtp_contentGroup,"index","group"),u);L(Bu(y.vtp_dimension,"index","dimension"),v);L(Bu(y.vtp_metric,"index","metric"),w);var x=L(y);x.vtp_fieldsToSet=void 0;x.vtp_contentGroup=void 0;x.vtp_dimension=void 0;x.vtp_metric=void 0;q=L(q,x)}L(Bu(q.vtp_contentGroup,"index","group"),u);L(Bu(q.vtp_dimension,"index","dimension"),v);L(Bu(q.vtp_metric,"index","metric"),w);var z=m(q),B=Ml(q.vtp_functionName);if(Ja(B)){var C="",E="";q.vtp_setTrackerName&&"string"==typeof q.vtp_trackerName?
""!==q.vtp_trackerName&&(E=q.vtp_trackerName,C=E+"."):(E="gtm"+wi(),C=E+".");var D=function(Ca){var ka=[].slice.call(arguments,0);ka[0]=C+ka[0];B.apply(window,ka)},I=function(Ca,ka){return void 0===ka?ka:Ca(ka)},R=function(Ca,ka){if(ka)for(var fb in ka)ka.hasOwnProperty(fb)&&D("set",Ca+fb,ka[fb])},Q=function(){},T={name:E};l(z,T,!0);var S=q.vtp_trackingId||t.trackingId;B("create",S,T);D("set","&gtm",pn(!0));
var X=!1;Qg()&&!X&&(D("set","&gcs",gh()),b(S,q,E));z._x_19&&z._x_20&&!d[E]&&(d[E]=!0,B(Rl(E,String(z._x_20))));q.vtp_enableRecaptcha&&D("require","recaptcha","recaptcha.js");(function(Ca,ka){void 0!==q[ka]&&D("set",Ca,q[ka])})("nonInteraction","vtp_nonInteraction");R("contentGroup",u);R("dimension",v);R("metric",w);var K={};l(z,K,!1)&&D("set",K);var U;q.vtp_enableLinkId&&D("require","linkid","linkid.js");D("set","hitCallback",function(){var Ca=z&&z.hitCallback;Ja(Ca)&&Ca();q.vtp_gtmOnSuccess()});var ba=function(Ca,ka){return void 0===q[Ca]?t[ka]:q[Ca]};if("TRACK_EVENT"==q.vtp_trackType){q.vtp_enableEcommerce&&(D("require","ec","ec.js"),Q());var O={hitType:"event",eventCategory:String(ba("vtp_eventCategory",
"category")),eventAction:String(ba("vtp_eventAction","action")),eventLabel:I(String,ba("vtp_eventLabel","label")),eventValue:I(Wa,ba("vtp_eventValue","value"))};l(U,O,!1);D("send",O);}else if("TRACK_SOCIAL"==q.vtp_trackType){}else if("TRACK_TRANSACTION"==q.vtp_trackType){}else if("TRACK_TIMING"==q.vtp_trackType){}else if("DECORATE_LINK"==q.vtp_trackType){}else if("DECORATE_FORM"==
q.vtp_trackType){}else if("TRACK_DATA"==q.vtp_trackType){}else{q.vtp_enableEcommerce&&(D("require","ec","ec.js"),Q());if(q.vtp_doubleClick||"DISPLAY_FEATURES"==q.vtp_advertisingFeaturesType){var md="_dc_gtm_"+String(q.vtp_trackingId).replace(/[^A-Za-z0-9-]/g,"");D("require","displayfeatures",void 0,{cookieName:md})}if("DISPLAY_FEATURES_WITH_REMARKETING_LISTS"==q.vtp_advertisingFeaturesType){var Ug="_dc_gtm_"+String(q.vtp_trackingId).replace(/[^A-Za-z0-9-]/g,
"");D("require","adfeatures",{cookieName:Ug})}U?D("send","pageview",U):D("send","pageview");Xa(z.urlPassthrough)&&Ol(C)}if(!c){var nd=q.vtp_useDebugVersion?"u/analytics_debug.js":"analytics.js";q.vtp_useInternalVersion&&!q.vtp_useDebugVersion&&(nd="internal/"+nd);
c=!0;var $d=Ym(z._x_19,"/analytics.js"),ae=Sk("https:","http:","//www.google-analytics.com/"+nd,z&&!!z.forceSSL);P("analytics.js"===nd&&$d?$d:ae,function(){var Ca=Kl();Ca&&Ca.loaded||q.vtp_gtmOnFailure();},q.vtp_gtmOnFailure)}}else J(q.vtp_gtmOnFailure)};(function(q){Y.__ua=q;Y.__ua.m="ua";Y.__ua.o=!0;Y.__ua.priorityOverride=0})(function(q){jh(function(){p(q)},[N.H,N.D])})}();

Y.h.inject_script=["google"],function(){function a(b,c){return{url:c}}(function(b){Y.__inject_script=b;Y.__inject_script.m="inject_script";Y.__inject_script.o=!0;Y.__inject_script.priorityOverride=0})(function(b){var c=b.vtp_urls||[],d=b.vtp_createPermissionError;return{assert:function(e,f){if(!n(f))throw d(e,{},"Script URL must be a string.");try{if(Pf(Eh(f),c))return}catch(g){throw d(e,{},"Invalid script URL filter.");}throw d(e,{},"Prohibited script URL: "+f);},T:a}})}();


Y.h.opt=["google"],function(){function a(l){return fh(l)}var b,c={name:!0,clientId:!0,sampleRate:!0,siteSpeedSampleRate:!0,alwaysSendReferrer:!0,allowAnchor:!0,allowLinker:!0,cookieName:!0,cookieDomain:!0,cookieExpires:!0,cookiePath:!0,cookieUpdate:!0,cookieFlags:!0,legacyCookieDomain:!0,legacyHistoryImport:!0,storage:!0,useAmpClientId:!0,storeGac:!0,_cd2l:!0,_useUp:!0,_cs:!0},d={allowAnchor:!0,allowLinker:!0,alwaysSendReferrer:!0,anonymizeIp:!0,cookieUpdate:!0,exFatal:!0,forceSSL:!0,javaEnabled:!0,
legacyHistoryImport:!0,nonInteraction:!0,useAmpClientId:!0,useBeacon:!0,storeGac:!0,allowAdFeatures:!0,allowAdPersonalizationSignals:!0,_cd2l:!0},e={urlPassthrough:!0},f=function(l,m,p){var q=0;if(l)for(var r in l)if(!e[r]&&l.hasOwnProperty(r)&&(p&&c[r]||!p&&void 0===c[r])){var t=d[r]?Xa(l[r]):l[r];"anonymizeIp"!=r||t||(t=void 0);m[r]=t;q++}return q},g=function(l){var m={};l.vtp_gaSettings&&L(Bu(l.vtp_gaSettings.vtp_fieldsToSet,"fieldName","value"),m);L(Bu(l.vtp_fieldsToSet,"fieldName","value"),m);
fh(N.H)||(m.storage="none");fh(N.D)||(m.allowAdFeatures=!1,m.storeGac=!1);bp()||(m.allowAdFeatures=!1);ap()||(m.allowAdPersonalizationSignals=!1);l.vtp_transportUrl&&(m._x_19=l.vtp_transportUrl);if(Xa(m.urlPassthrough)){m._useUp=!0;var p=!1;Qg()&&!p&&(m._cs=a)}return m},h=function(l){if(l.vtp_gaSettings){var m=L(l.vtp_gaSettings);m.vtp_fieldsToSet=void 0;l=L(l,m)}var p=g(l),q=Ml(l.vtp_functionName);if(Ja(q)){q.r=!0;var r="",t="";l.vtp_setTrackerName&&"string"===typeof l.vtp_trackerName?""!==l.vtp_trackerName&&(t=l.vtp_trackerName,r=t+"."):(t="gtm"+wi(),r=t+".");var u={name:t};f(p,u,!0);var v={"&gtm":pn(!0)};f(p,v,!1);
var w=encodeURI(Sk("https:","http:","//www.google-analytics.com/"+(l.vtp_useDebugVersion?"u/analytics_debug.js":"analytics.js"),!!p.forceSSL));q("create",l.vtp_trackingId,u);q(r+"set",v);q(r+"require",l.vtp_optimizeContainerId,{dataLayer:"dataLayer"});q(l.vtp_gtmOnSuccess);q(r+"require","render");b||(b=!0,P(w,function(){return Kl().loaded||l.vtp_gtmOnFailure()},l.vtp_gtmOnFailure));var y=V("dataLayer"),x=y&&y.hide;x&&(x.end||!0===x["GTM-M6FZ495"])&&(x[l.vtp_optimizeContainerId]=!0)}else J(l.vtp_gtmOnFailure)};
(function(l){Y.__opt=l;Y.__opt.m="opt";Y.__opt.o=!0;Y.__opt.priorityOverride=0})(function(l){jh(function(){h(l)},[N.H,N.D])})}();




Y.h.aev=["google"],function(){function a(t,u,v){var w=t||Ji(u,"gtm");if(w)return w[v]}function b(t,u,v,w,y){y||(y="element");var x=u+"."+v,z;if(p.hasOwnProperty(x))z=p[x];else{var B=a(t,u,y);if(B&&(z=w(B),p[x]=z,q.push(x),35<q.length)){var C=q.shift();delete p[C]}}return z}function c(t,u,v,w){var y=a(t,u,r[v]);return void 0!==y?y:w}function d(t,u){if(!t)return!1;var v=e(Ft());Na(u)||(u=String(u||"").replace(/\s+/g,"").split(","));for(var w=[v],y=0;y<u.length;y++){var x=u[y];if(x.hasOwnProperty("is_regex"))if(x.is_regex)try{x=
new RegExp(x.domain)}catch(B){continue}else x=x.domain;if(x instanceof RegExp){if(x.test(t))return!1}else{var z=x;if(0!=z.length){if(0<=e(t).indexOf(z))return!1;w.push(e(z))}}}return!Au(t,w)}function e(t){m.test(t)||(t="http://"+t);return Ch(Eh(t),"HOST",!0)}function f(t,u,v,w){switch(t){case "SUBMIT_TEXT":return b(u,v,"FORM."+t,g,"formSubmitElement")||w;case "LENGTH":var y=b(u,v,"FORM."+t,h);return void 0===y?w:y;case "INTERACTED_FIELD_ID":return l(u,v,"id",w);case "INTERACTED_FIELD_NAME":return l(u,
v,"name",w);case "INTERACTED_FIELD_TYPE":return l(u,v,"type",w);case "INTERACTED_FIELD_POSITION":var x=a(u,v,"interactedFormFieldPosition");return void 0===x?w:x;case "INTERACT_SEQUENCE_NUMBER":var z=a(u,v,"interactSequenceNumber");return void 0===z?w:z;default:return w}}function g(t){switch(t.tagName.toLowerCase()){case "input":return kc(t,"value");case "button":return lc(t);default:return null}}function h(t){if("form"===t.tagName.toLowerCase()&&t.elements){for(var u=0,v=0;v<t.elements.length;v++)aq(t.elements[v])&&
u++;return u}}function l(t,u,v,w){var y=a(t,u,"interactedFormField");return y&&kc(y,v)||w}var m=/^https?:\/\//i,p={},q=[],r={ATTRIBUTE:"elementAttribute",CLASSES:"elementClasses",ELEMENT:"element",ID:"elementId",HISTORY_CHANGE_SOURCE:"historyChangeSource",HISTORY_NEW_STATE:"newHistoryState",HISTORY_NEW_URL_FRAGMENT:"newUrlFragment",HISTORY_OLD_STATE:"oldHistoryState",HISTORY_OLD_URL_FRAGMENT:"oldUrlFragment",TARGET:"elementTarget"};(function(t){Y.__aev=t;Y.__aev.m="aev";Y.__aev.o=!0;Y.__aev.priorityOverride=
0})(function(t){var u=t.vtp_gtmEventId,v=t.vtp_defaultValue,w=t.vtp_varType,y;t.vtp_gtmCachedValues&&(y=t.vtp_gtmCachedValues.gtm);switch(w){case "TAG_NAME":var x=a(y,u,"element");return x&&x.tagName||v;case "TEXT":return b(y,u,w,lc)||v;case "URL":var z;a:{var B=String(a(y,u,"elementUrl")||v||""),C=Eh(B),E=String(t.vtp_component||"URL");switch(E){case "URL":z=B;break a;case "IS_OUTBOUND":z=
d(B,t.vtp_affiliatedDomains);break a;default:z=Ch(C,E,t.vtp_stripWww,t.vtp_defaultPages,t.vtp_queryKey)}}return z;case "ATTRIBUTE":var D;if(void 0===t.vtp_attribute)D=c(y,u,w,v);else{var I=t.vtp_attribute,R=a(y,u,"element");D=R&&kc(R,I)||v||""}return D;case "MD":var Q=t.vtp_mdValue,T=b(y,u,"MD",yt);return Q&&T?Bt(T,Q)||v:T||v;case "FORM":return f(String(t.vtp_component||"SUBMIT_TEXT"),y,u,v);default:var S=c(y,u,w,v);Rt(S,"aev",t.vtp_gtmEventId);return S}})}();

Y.h.gas=["google"],function(){(function(a){Y.__gas=a;Y.__gas.m="gas";Y.__gas.o=!0;Y.__gas.priorityOverride=0})(function(a){var b=L(a),c=b;c[ne.kb]=null;c[ne.yh]=null;var d=b=c;d.vtp_fieldsToSet=d.vtp_fieldsToSet||[];var e=d.vtp_cookieDomain;void 0!==e&&(d.vtp_fieldsToSet.push({fieldName:"cookieDomain",value:e}),delete d.vtp_cookieDomain);return b})}();
Y.h.fsl=[],function(){function a(){var e=V("document"),f=c(),g=HTMLFormElement.prototype.submit;hc(e,"click",function(h){var l=h.target;if(l&&(l=nc(l,["button","input"],100))&&("submit"==l.type||"image"==l.type)&&l.name&&kc(l,"value")){var m;l.form?l.form.tagName?m=l.form:m=H.getElementById(l.form):m=nc(l,["form"],100);m&&f.store(m,l)}},!1);hc(e,"submit",function(h){var l=h.target;if(!l)return h.returnValue;var m=h.defaultPrevented||!1===h.returnValue,p=b(l)&&!m,q=f.get(l),r=!0;if(d(l,function(){if(r){var t;
q&&(t=e.createElement("input"),t.type="hidden",t.name=q.name,t.value=q.value,l.appendChild(t));g.call(l);t&&l.removeChild(t)}},m,p,q))r=!1;else return m||(h.preventDefault&&h.preventDefault(),h.returnValue=!1),!1;return h.returnValue},!1);HTMLFormElement.prototype.submit=function(){var h=this,l=b(h),m=!0;d(h,function(){m&&g.call(h)},!1,l)&&(g.call(h),m=!1)}}function b(e){var f=e.target;return f&&"_self"!==f&&"_parent"!==f&&"_top"!==f?!1:!0}function c(){var e=[],f=function(g){return Qa(e,function(h){return h.form===
g})};return{store:function(g,h){var l=f(g);l?l.button=h:e.push({form:g,button:h})},get:function(g){var h=f(g);return h?h.button:null}}}function d(e,f,g,h,l){var m=Yp("fsl",g?"nv.mwt":"mwt",0),p;p=g?Yp("fsl","nv.ids",[]):Yp("fsl","ids",[]);if(!p.length)return!0;var q=Up(e,"gtm.formSubmit",p),r=e.action;r&&r.tagName&&(r=e.cloneNode(!1).action);q["gtm.elementUrl"]=r;l&&(q["gtm.formSubmitElement"]=l);if(h&&m){if(!Jt(q,it(f),m))return!1}else Jt(q,function(){},m||2E3);return!0}(function(e){Y.__fsl=e;Y.__fsl.m=
"fsl";Y.__fsl.o=!0;Y.__fsl.priorityOverride=0})(function(e){var f=e.vtp_waitForTags,g=e.vtp_checkValidation,h=Number(e.vtp_waitForTagsTimeout);if(!h||0>=h)h=2E3;var l=e.vtp_uniqueTriggerId||"0";if(f){var m=function(q){return Math.max(h,q)};Xp("fsl","mwt",m,0);g||Xp("fsl","nv.mwt",m,0)}var p=function(q){q.push(l);return q};Xp("fsl","ids",p,[]);g||Xp("fsl","nv.ids",p,[]);Ot("fsl")||(a(),Pt("fsl"));J(e.vtp_gtmOnSuccess)})}();Y.h.smm=["google"],function(){(function(a){Y.__smm=a;Y.__smm.m="smm";Y.__smm.o=!0;Y.__smm.priorityOverride=0})(function(a){var b=a.vtp_input,c=Bu(a.vtp_map,"key","value")||{},d=c.hasOwnProperty(b)?c[b]:a.vtp_defaultValue;Rt(d,"smm",a.vtp_gtmEventId);return d})}();






Y.h.html=["customScripts"],function(){function a(d,e,f,g){return function(){try{if(0<e.length){var h=e.shift(),l=a(d,e,f,g);if("SCRIPT"==String(h.nodeName).toUpperCase()&&"text/gtmscript"==h.type){var m=H.createElement("script");m.async=!1;m.type="text/javascript";m.id=h.id;m.text=h.text||h.textContent||h.innerHTML||"";h.charset&&(m.charset=h.charset);var p=h.getAttribute("data-gtmsrc");p&&(m.src=p,bc(m,l));d.insertBefore(m,null);p||l()}else if(h.innerHTML&&0<=h.innerHTML.toLowerCase().indexOf("<script")){for(var q=
[];h.firstChild;)q.push(h.removeChild(h.firstChild));d.insertBefore(h,null);a(h,q,l,g)()}else d.insertBefore(h,null),l()}else f()}catch(r){J(g)}}}var c=function(d){if(H.body){var e=
d.vtp_gtmOnFailure,f=St(d.vtp_html,d.vtp_gtmOnSuccess,e),g=f.ii,h=f.onSuccess;if(d.vtp_useIframe){}else d.vtp_supportDocumentWrite?b(g,h,e):a(H.body,mc(g),h,e)()}else Dt(function(){c(d)},
200)};Y.__html=c;Y.__html.m="html";Y.__html.o=!0;Y.__html.priorityOverride=0}();






Y.h.lcl=[],function(){function a(){var c=V("document"),d=0,e=function(f){var g=f.target;if(g&&3!==f.which&&!(f.qg||f.timeStamp&&f.timeStamp===d)){d=f.timeStamp;g=nc(g,["a","area"],100);if(!g)return f.returnValue;var h=f.defaultPrevented||!1===f.returnValue,l=Yp("lcl",h?"nv.mwt":"mwt",0),m;m=h?Yp("lcl","nv.ids",[]):Yp("lcl","ids",[]);if(m.length){var p=Up(g,"gtm.linkClick",m);if(b(f,g,c)&&!h&&l&&g.href){var q=!!Qa(String(pc(g,"rel")||"").split(" "),function(u){return"noreferrer"===u.toLowerCase()});
q&&yg(36);var r=V((pc(g,"target")||"_self").substring(1)),t=!0;if(Jt(p,it(function(){var u;if(u=t&&r){var v;a:if(q){var w;try{w=new MouseEvent(f.type,{bubbles:!0})}catch(y){if(!c.createEvent){v=!1;break a}w=c.createEvent("MouseEvents");w.initEvent(f.type,!0,!0)}w.qg=!0;f.target.dispatchEvent(w);v=!0}else v=!1;u=!v}u&&(r.location.href=pc(g,"href"))}),l))t=!1;else return f.preventDefault&&f.preventDefault(),f.returnValue=!1}else Jt(p,function(){},l||2E3);return!0}}};hc(c,"click",e,!1);hc(c,"auxclick",
e,!1)}function b(c,d,e){if(2===c.which||c.ctrlKey||c.shiftKey||c.altKey||c.metaKey)return!1;var f=pc(d,"href"),g=f.indexOf("#"),h=pc(d,"target");if(h&&"_self"!==h&&"_parent"!==h&&"_top"!==h||0===g)return!1;if(0<g){var l=Ht(f),m=Ht(e.location);return l!==m}return!0}(function(c){Y.__lcl=c;Y.__lcl.m="lcl";Y.__lcl.o=!0;Y.__lcl.priorityOverride=0})(function(c){var d=void 0===c.vtp_waitForTags?!0:c.vtp_waitForTags,e=void 0===c.vtp_checkValidation?!0:c.vtp_checkValidation,f=Number(c.vtp_waitForTagsTimeout);
if(!f||0>=f)f=2E3;var g=c.vtp_uniqueTriggerId||"0";if(d){var h=function(m){return Math.max(f,m)};Xp("lcl","mwt",h,0);e||Xp("lcl","nv.mwt",h,0)}var l=function(m){m.push(g);return m};Xp("lcl","ids",l,[]);e||Xp("lcl","nv.ids",l,[]);Ot("lcl")||(a(),Pt("lcl"));J(c.vtp_gtmOnSuccess)})}();
Y.h.evl=["google"],function(){function a(){var f=Number(It("gtm.start"))||0;return Et().getTime()-f}function b(f,g,h,l){function m(){if(!ph(f.target)){g.has(d.ad)||g.set(d.ad,""+a());g.has(d.$d)||g.set(d.$d,""+a());var q=0;g.has(d.dd)&&(q=Number(g.get(d.dd)));q+=100;g.set(d.dd,""+q);if(q>=h){var r=Up(f.target,"gtm.elementVisibility",[g.g]),t=rh(f.target);r["gtm.visibleRatio"]=Math.round(1E3*t)/10;r["gtm.visibleTime"]=h;r["gtm.visibleFirstTime"]=Number(g.get(d.$d));r["gtm.visibleLastTime"]=Number(g.get(d.ad));
Jt(r);l()}}}if(!g.has(d.Yb)&&(0==h&&m(),!g.has(d.Eb))){var p=V("self").setInterval(m,100);g.set(d.Yb,p)}}function c(f){f.has(d.Yb)&&(V("self").clearInterval(Number(f.get(d.Yb))),f.s(d.Yb))}var d={Yb:"polling-id-",$d:"first-on-screen-",ad:"recent-on-screen-",dd:"total-visible-time-",Eb:"has-fired-"},e=function(f,g){this.element=f;this.g=g};e.prototype.has=function(f){return!!this.element.getAttribute("data-gtm-vis-"+f+this.g)};e.prototype.get=function(f){return this.element.getAttribute("data-gtm-vis-"+
f+this.g)};e.prototype.set=function(f,g){this.element.setAttribute("data-gtm-vis-"+f+this.g,g)};e.prototype.s=function(f){this.element.removeAttribute("data-gtm-vis-"+f+this.g)};(function(f){Y.__evl=f;Y.__evl.m="evl";Y.__evl.o=!0;Y.__evl.priorityOverride=0})(function(f){function g(){var y=!1,x=null;if("CSS"===l){try{x=lh(m)}catch(D){yg(46)}y=!!x&&v.length!=x.length}else if("ID"===l){var z=H.getElementById(m);z&&(x=[z],y=1!=v.length||v[0]!==z)}x||(x=[],y=0<v.length);if(y){for(var B=0;B<v.length;B++){var C=
new e(v[B],t);c(C)}v=[];for(var E=0;E<x.length;E++)v.push(x[E]);0<=w&&xh(w);0<v.length&&(w=wh(h,v,[r]))}}function h(y){var x=new e(y.target,t);y.intersectionRatio>=r?x.has(d.Eb)||b(y,x,q,"ONCE"===u?function(){for(var z=0;z<v.length;z++){var B=new e(v[z],t);B.set(d.Eb,"1");c(B)}xh(w);if(p&&Wq)for(var C=0;C<Wq.length;C++)Wq[C]===g&&Wq.splice(C,1)}:function(){x.set(d.Eb,"1");c(x)}):(c(x),"MANY_PER_ELEMENT"===u&&x.has(d.Eb)&&(x.s(d.Eb),x.s(d.dd)),x.s(d.ad))}var l=f.vtp_selectorType,m;"ID"===l?m=String(f.vtp_elementId):
"CSS"===l&&(m=String(f.vtp_elementSelector));var p=!!f.vtp_useDomChangeListener,q=f.vtp_useOnScreenDuration&&Number(f.vtp_onScreenDuration)||0,r=(Number(f.vtp_onScreenRatio)||50)/100,t=f.vtp_uniqueTriggerId,u=f.vtp_firingFrequency,v=[],w=-1;g();p&&Xq(g);J(f.vtp_gtmOnSuccess)})}();


var wx={};wx.macro=function(a){if(pt.je.hasOwnProperty(a))return pt.je[a]},wx.onHtmlSuccess=pt.gg(!0),wx.onHtmlFailure=pt.gg(!1);wx.dataLayer=Di;wx.callback=function(a){ui.hasOwnProperty(a)&&Ja(ui[a])&&ui[a]();delete ui[a]};wx.bootstrap=0;wx._spx=!1;function xx(){ji[kf.M]=wx;gb(vi,Y.h);Re=Re||pt;Se=ff}
function yx(){var a=!1;a&&Gl("INIT");Eg().s();ji=A.google_tag_manager=A.google_tag_manager||{};Sn();
Wj.enable_gbraid_cookie_write=!0;if(ji[kf.M]){var b=ji.zones;b&&b.unregisterChild(kf.M);}else{for(var c=data.resource||{},d=c.macros||[],e=0;e<d.length;e++)Ke.push(d[e]);for(var f=c.tags||[],g=0;g<f.length;g++)Ne.push(f[g]);for(var h=c.predicates||[],l=0;l<h.length;l++)Me.push(h[l]);for(var m=c.rules||[],p=0;p<m.length;p++){for(var q=m[p],r={},t=
0;t<q.length;t++)r[q[t][0]]=Array.prototype.slice.call(q[t],1);Le.push(r)}Pe=Y;Qe=su;var u=data.permissions||{},v=data.sandboxed_scripts,w=data.security_groups;vs();pf=new of(u);if(void 0!==v)for(var y=["sandboxedScripts"],x=0;x<v.length;x++){var z=v[x].replace(/^_*/,"");vi[z]=y}ys(w);xx();ot();tl=!1;ul=0;if("interactive"==H.readyState&&!H.createEventObject||"complete"==H.readyState)wl();else{hc(H,"DOMContentLoaded",wl);hc(H,"readystatechange",wl);if(H.createEventObject&&H.documentElement.doScroll){var B=
!0;try{B=!A.frameElement}catch(R){}B&&xl()}hc(A,"load",wl)}rr=!1;"complete"===H.readyState?tr():hc(A,"load",tr);Am&&A.setInterval(vm,864E5);
si=(new Date).getTime();if(a){var I=Hl("INIT");
}}}
(function(a){if(!A["__TAGGY_INSTALLED"]){var b=!1;if(H.referrer){var c=Eh(H.referrer);b="cct.google"===Bh(c,"host")}if(!b){var d=Ti("googTaggyReferrer");b=d.length&&d[0].length}b&&(A["__TAGGY_INSTALLED"]=!0,dc("https://cct.google/taggy/agent.js"))}var f=function(){var m=A["google.tagmanager.debugui2.queue"];m||(m=[],A["google.tagmanager.debugui2.queue"]=m,dc("https://www.googletagmanager.com/debug/bootstrap"));var p={messageType:"CONTAINER_STARTING",data:{scriptSource:Zb,containerProduct:"GTM",debug:!1}};p.data.resume=function(){a()};kf.Og&&(p.data.initialPublish=!0);m.push(p)},g="x"===Ch(A.location,"query",!1,void 0,"gtm_debug");if(!g&&H.referrer){var h=Eh(H.referrer);g="tagassistant.google.com"===Bh(h,"host")}if(!g){var l=Ti("__TAG_ASSISTANT");g=l.length&&l[0].length}A.__TAG_ASSISTANT_API&&(g=!0);g&&Zb?f():a()})(yx);

})()
