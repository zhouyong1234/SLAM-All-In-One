/***
 * https://github.com/toddmotto/echo/blob/master/src/echo.js
 */
(function (root, factory) {
    if (typeof define === 'function' && define.amd) {
        define(function () {
            return factory(root);
        });
    } else if (typeof exports === 'object') {
        module.exports = factory;
    } else {
        root.echo = factory(root);
    }
})(this, function (root) {

        'use strict';

        var echo = {};

        var callback = function () {
        };

        var offset, poll, delay, useDebounce, unload;

        var isHidden = function (element) {
            return (element.offsetParent === null);
        };

        var inView = function (element, view) {
            if (isHidden(element)) {
                return false;
            }

            var box = element.getBoundingClientRect();
            return (box.right >= view.l && box.bottom >= view.t && box.left <= view.r && box.top <= view.b);
        };

        var debounceOrThrottle = function () {
            if (!useDebounce && !!poll) {
                return;
            }
            clearTimeout(poll);
            poll = setTimeout(function () {
                echo.render();
                poll = null;
            }, delay);
        };

        var imgurl = function (d, s, t) {
            var _id = parseInt(d);
            var _a = _id / 1000000;
            var _b = "";
            if (_a >= 4 && _a < 8) {
                _b = "2";
            } else if (_a >= 8 && _a < 12) {
                _b = "3";
            } else if (_a >= 12) {
                _b = "4";
            }

            var _c = 12 - d.length;
            var _fd = d;
            for (var i = 0; i < _c; i++) {
                _fd = "0" + _fd;
            }
            var _u = "https://ewr1.vultrobjects.com/imgspice" + _b + "/" + _fd.substr(0, 3) + "/" + _fd.substr(3, 3) + "/" + _fd.substr(6, 3) + "/" + _fd.substr(9, 3) + "_" + s + "." + t;
            return _u;
        }

        echo.init = function (opts) {
            opts = opts || {};
            var offsetAll = opts.offset || 0;
            var offsetVertical = opts.offsetVertical || offsetAll;
            var offsetHorizontal = opts.offsetHorizontal || offsetAll;
            var optionToInt = function (opt, fallback) {
                return parseInt(opt || fallback, 10);
            };
            offset = {
                t: optionToInt(opts.offsetTop, offsetVertical),
                b: optionToInt(opts.offsetBottom, offsetVertical),
                l: optionToInt(opts.offsetLeft, offsetHorizontal),
                r: optionToInt(opts.offsetRight, offsetHorizontal)
            };
            delay = optionToInt(opts.throttle, 250);
            useDebounce = opts.debounce !== false;
            unload = !!opts.unload;
            callback = opts.callback || callback;
            echo.render();
            if (document.addEventListener) {
                root.addEventListener('scroll', debounceOrThrottle, false);
                root.addEventListener('load', debounceOrThrottle, false);
            } else {
                root.attachEvent('onscroll', debounceOrThrottle);
                root.attachEvent('onload', debounceOrThrottle);
            }
        };

        echo.render = function (context) {
            var nodes = (context || document).querySelectorAll('[d]');
            var length = nodes.length;
            var src, elem;
            var view = {
                l: 0 - offset.l,
                t: 0 - offset.t,
                b: (root.innerHeight || document.documentElement.clientHeight) + offset.b,
                r: (root.innerWidth || document.documentElement.clientWidth) + offset.r
            };
            for (var i = 0; i < length; i++) {
                elem = nodes[i];
                if (inView(elem, view)) {

                    if (unload) {
                        elem.setAttribute('data-echo-placeholder', elem.src);
                    }

                    var _d = elem.getAttribute("d");
                    var _s = elem.getAttribute("s");
                    var _t = elem.getAttribute("t");
                    var _r = elem.getAttribute("r");
                    if (_d == null || _s == null || _t == null) {
                        continue;
                    }
                    if (_r != null && _r == 'true') {
                        continue;
                    }
                    var data_echo = imgurl(_d, _s, _t);
                    if (elem.src !== (src = data_echo)) {
                        elem.src = src;
                        elem.setAttribute("r", "true");
                    }

                    if (!unload) {
                        elem.removeAttribute('data-echo');
                        elem.removeAttribute('data-echo-background');
                    }

                    callback(elem, 'load');
                }
                else if (unload && !!(src = elem.getAttribute('data-echo-placeholder'))) {

                    if (elem.getAttribute('data-echo-background') !== null) {
                        elem.style.backgroundImage = 'url(' + src + ')';
                    }
                    else {
                        elem.src = src;
                    }

                    elem.removeAttribute('data-echo-placeholder');
                    callback(elem, 'unload');
                }
            }
            if (!length) {
                echo.detach();
            }
        };

        echo.detach = function () {
            if (document.removeEventListener) {
                root.removeEventListener('scroll', debounceOrThrottle);
            } else {
                root.detachEvent('onscroll', debounceOrThrottle);
            }
            clearTimeout(poll);
        };

        return echo;

    }
);