"use strict";var precacheConfig=[["/dash/index.html","210bc0c73066fbf8a39c3c19e1f826b9"],["/dash/static/css/main.6e7549cd.css","9fb74ac5a50e4e8ee3527e61e223d464"],["/dash/static/js/main.60e806b2.js","8e46e91081e42e3819a9991a80614136"],["/dash/static/media/add.74fc797d.svg","74fc797d2d34f79f05a5255671eaeb25"],["/dash/static/media/chart.40188f26.svg","40188f26016eb88a9ba623cda0399179"],["/dash/static/media/close.09e70abb.svg","09e70abbf7f3c8e0a712186b7840f08d"],["/dash/static/media/expand_less.f146d110.svg","f146d110519d5e606fccc3133279ebc0"],["/dash/static/media/expand_more.790f345a.svg","790f345a092e18853b1b54600eb72753"],["/dash/static/media/field.ab2bbfb1.png","ab2bbfb1df961a74c354e2f3084803e2"],["/dash/static/media/refresh.28eb3190.svg","28eb319041e3f0c992987613f4ac1a72"],["/dash/static/media/remove.db082a01.svg","db082a01d74cfb909df8fcc377399583"],["/dash/static/media/roboto-v18-latin-regular.372d0cc3.ttf","372d0cc3288fe8e97df49742baefce90"],["/dash/static/media/roboto-v18-latin-regular.5d4aeb4e.woff2","5d4aeb4e5f5ef754e307d7ffaef688bd"],["/dash/static/media/roboto-v18-latin-regular.68889c24.eot","68889c246da2739681c1065d15a1ab0b"],["/dash/static/media/roboto-v18-latin-regular.8681f434.svg","8681f434273fd6a267b1a16a035c5f79"],["/dash/static/media/roboto-v18-latin-regular.bafb105b.woff","bafb105baeb22d965c70fe52ba6b49d9"],["/dash/static/media/save.860e11ac.svg","860e11acc4964d354baa623516c7848a"],["/dash/static/media/wifi.e88e94c6.svg","e88e94c63f7ef38a03e506783257c420"],["/dash/static/media/wifi_off.d8944708.svg","d8944708cef0950974cd1f9265c6fb9e"]],cacheName="sw-precache-v3-sw-precache-webpack-plugin-"+(self.registration?self.registration.scope:""),ignoreUrlParametersMatching=[/^utm_/],addDirectoryIndex=function(e,a){var t=new URL(e);return"/"===t.pathname.slice(-1)&&(t.pathname+=a),t.toString()},cleanResponse=function(a){return a.redirected?("body"in a?Promise.resolve(a.body):a.blob()).then(function(e){return new Response(e,{headers:a.headers,status:a.status,statusText:a.statusText})}):Promise.resolve(a)},createCacheKey=function(e,a,t,n){var r=new URL(e);return n&&r.pathname.match(n)||(r.search+=(r.search?"&":"")+encodeURIComponent(a)+"="+encodeURIComponent(t)),r.toString()},isPathWhitelisted=function(e,a){if(0===e.length)return!0;var t=new URL(a).pathname;return e.some(function(e){return t.match(e)})},stripIgnoredUrlParameters=function(e,t){var a=new URL(e);return a.hash="",a.search=a.search.slice(1).split("&").map(function(e){return e.split("=")}).filter(function(a){return t.every(function(e){return!e.test(a[0])})}).map(function(e){return e.join("=")}).join("&"),a.toString()},hashParamName="_sw-precache",urlsToCacheKeys=new Map(precacheConfig.map(function(e){var a=e[0],t=e[1],n=new URL(a,self.location),r=createCacheKey(n,hashParamName,t,/\.\w{8}\./);return[n.toString(),r]}));function setOfCachedUrls(e){return e.keys().then(function(e){return e.map(function(e){return e.url})}).then(function(e){return new Set(e)})}self.addEventListener("install",function(e){e.waitUntil(caches.open(cacheName).then(function(n){return setOfCachedUrls(n).then(function(t){return Promise.all(Array.from(urlsToCacheKeys.values()).map(function(a){if(!t.has(a)){var e=new Request(a,{credentials:"same-origin"});return fetch(e).then(function(e){if(!e.ok)throw new Error("Request for "+a+" returned a response with status "+e.status);return cleanResponse(e).then(function(e){return n.put(a,e)})})}}))})}).then(function(){return self.skipWaiting()}))}),self.addEventListener("activate",function(e){var t=new Set(urlsToCacheKeys.values());e.waitUntil(caches.open(cacheName).then(function(a){return a.keys().then(function(e){return Promise.all(e.map(function(e){if(!t.has(e.url))return a.delete(e)}))})}).then(function(){return self.clients.claim()}))}),self.addEventListener("fetch",function(a){if("GET"===a.request.method){var e,t=stripIgnoredUrlParameters(a.request.url,ignoreUrlParametersMatching),n="index.html";(e=urlsToCacheKeys.has(t))||(t=addDirectoryIndex(t,n),e=urlsToCacheKeys.has(t));var r="/dash/index.html";!e&&"navigate"===a.request.mode&&isPathWhitelisted(["^(?!\\/__).*"],a.request.url)&&(t=new URL(r,self.location).toString(),e=urlsToCacheKeys.has(t)),e&&a.respondWith(caches.open(cacheName).then(function(e){return e.match(urlsToCacheKeys.get(t)).then(function(e){if(e)return e;throw Error("The cached response that was expected is missing.")})}).catch(function(e){return console.warn('Couldn\'t serve response for "%s" from cache: %O',a.request.url,e),fetch(a.request)}))}});