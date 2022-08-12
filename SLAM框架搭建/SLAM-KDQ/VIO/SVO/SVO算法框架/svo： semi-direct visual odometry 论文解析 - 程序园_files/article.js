$(function () {
	$("a[class=outlink]").each(function(){
		$(this).hover(function(){
			$this = $(this);
			var link = $this.attr("data");
			if(link.indexOf("/link?url=#") != -1){
				link = link.replace("/link?url=" , "");
			}
	        $this.attr("href" , link);
		},function(){
			$this = $(this);
	        $this.attr("href" , "javascript:void()");
		});
	
	});
});