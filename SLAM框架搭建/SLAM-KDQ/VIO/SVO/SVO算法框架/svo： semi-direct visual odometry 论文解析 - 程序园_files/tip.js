//返回顶部
var mouseover_r;
var mouseout_r;
function show_return() {
    var $backToTopTxt = "返回顶部";
    var $backToTopEle = $('<div class="return"></div>');
    $backToTopEle.appendTo($("body")).attr("title", $backToTopTxt).css({
                                                                           //opacity: .4
                                                                       }).click(function () {
        $("html, body").animate({
                                    scrollTop: 0
                                }, 120);
    }), $backToTopFun = function () {
        var st = $(document).scrollTop(), winh = $(window).height();
        (st > 0) ? $backToTopEle.show() : $backToTopEle.hide();
        //IE6下的定位
        if (!window.XMLHttpRequest) {
            $backToTopEle.css("top", st + winh - 166);
        }
    };
    $(window).bind("scroll", $backToTopFun);
    $(function () {
        $backToTopFun();
    });
}

function show_op() {
    var note = document.getElementsByClassName("right_top")[0];
    var zone = $('.operate_zone');
    if (note && zone) {
        var screenPosition = note.getBoundingClientRect();
        var height = screenPosition.height;
        if (height < 100) {
            height = 100;
        }
        h = $(document).scrollTop();

        var head_h = 60; // 顶部
        var foot_h = 140; // 底部
        var doc_h = $(document).height(); // 文档的高度
        var zone_h = zone.height(); // fix的高度
        
        if(h > height + head_h){
        	// 滚动条的位置，大于right_top + head_h
        	if(zone_h + foot_h + head_h + h < doc_h){
        		zone.css('position','fixed').css('top',head_h);
        	}else{
        		var top_h = doc_h - h - zone_h - foot_h;
        		zone.css('position','fixed').css('top',top_h);
        	}
        	
        	$('.operate_zone').css('width' , $(note).css('width'));
        }else{
        	zone.css('position','inherit').css('top',0)
        }
    }
}

//延迟加载图片动画
$(document).ready(function ($) {
    $('a.btn-active').click(function () {
        return false;
    });
    show_return();
    $(window).bind("scroll", show_op);
    // $("img").lazyload({ placeholder: "../images/fill.gif", effect: "fadeIn" });
});

function close_sept() {
    $('.sept-notify').remove();
    SetCookie('show_sept', 1, 86400000 * 3);
}