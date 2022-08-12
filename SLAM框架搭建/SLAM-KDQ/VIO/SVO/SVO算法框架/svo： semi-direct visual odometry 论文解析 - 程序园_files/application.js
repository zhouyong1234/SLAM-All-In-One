function resize_article_image(e, a) {
    $(window).load(function () {
        $(e + " img").each(function (e, t) {
            var i = $(t), n = i.width(), s = i.height();
            n > a && (i.height(i.height() * a / n), i.width(a)), i.hasClass("alignCenter") || i.hasClass("alignLeft")
                                                                 || i.hasClass("alignLeft0") || (n > 280 || n > a || n
                                                                                                                     > 250
                                                                                                                     && s
                                                                                                                        > 300)
                                                                                                && i.css("display",
                                                                                                         "block")
        })
    })
}