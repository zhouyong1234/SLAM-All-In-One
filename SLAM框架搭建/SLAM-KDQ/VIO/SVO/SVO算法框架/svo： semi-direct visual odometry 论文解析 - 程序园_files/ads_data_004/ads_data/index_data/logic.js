/*clicktag*/
function addevents(){
	document.getElementById("bannerCTA").addEventListener("click", trackClick); 
	document.getElementById("disclaimer-copy").addEventListener("click", trackClick); 
	document.getElementById("bannerCTA").addEventListener("mouseover", movecta); 
    document.getElementById("bannerCTA").addEventListener("mouseout", moveCtaToOriginal); 
	document.getElementById("more-info").addEventListener("mouseover", showDisclaimer); 
	document.getElementById("disclaimer-copy").addEventListener("mouseout", hideDisclaimer); 
	
	
}

function trackClick(){
    window.open(clickTag,'_blank');
}
function movecta(){
	TweenLite.to(["#cta"], 0.2, {x:-5, ease: Quad.easeOut, delay:0});

}
function moveCtaToOriginal(){
	TweenLite.to(["#cta"], 0.2, {x:0, ease: Quad.easeOut, delay:0});

}
function showDisclaimer(){
	TweenLite.to(["#disclaimer-bg"], 0, {autoAlpha:1, ease: Quad.easeOut, delay:0});
	TweenLite.to(["#disclaimer-copy"], 0, {autoAlpha:1, zIndex:100, ease: Quad.easeOut, delay:0});
}
function hideDisclaimer(){
	TweenLite.to(["#disclaimer-bg"], 0, {autoAlpha:0, ease: Quad.easeOut, delay:0});
	TweenLite.to(["#disclaimer-copy"], 0, {autoAlpha:0, zIndex:1, ease: Quad.easeOut, delay:0});

}

function showvisibility(){
	TweenLite.set("#more-info", { autoAlpha:1});
}



function startAnimation(){
	
	TweenLite.set(["#delearName"],{x:300,alpha:1});	
	TweenLite.set(["#acura-logo",".ad"], { alpha:1});
	TweenLite.set(["#f2white_bg"],{x:300, alpha:1});

	TweenLite.to("#frame1_copy1", 0, {alpha: 1, delay:1});
	TweenLite.to("#frame1_copy2", 0, {alpha: 1, delay:1.5});
	TweenLite.to("#frame1_copy3", 0, {alpha: 1, delay:2});
	TweenLite.delayedCall(2.5,frame2Animation);
}
function frame2Animation(){
	TweenLite.set(["#frame2-bg", "#cta", "#acura-logo-2","#f2_ilx_logo","#frame3_copy1"],{x:300, alpha:1});
	TweenLite.to(["#acura-logo", "#frame1_copy1", "#frame1_copy2","#frame1_copy3"], 0.3, {alpha:0, delay:0.6});

	TweenLite.to(["#frame2-bg", "#cta", "#acura-logo-2","#f2_ilx_logo","#frame3_copy1","#f2white_bg"], 0.3, {x:0, ease: Quad.easeIn, delay:0});
	TweenLite.to("#frame2-bg", 3, {scale:1.1, ease: Quad.easeIn, delay:0.3});
	TweenLite.to(["#frame3_copy1"], 0.4, {x:-300, ease: Quad.easeIn, delay:3.3});
	
	TweenLite.delayedCall(3.5,frame3Animation);
}
function frame3Animation(){
	TweenLite.set(["#frame3_copy2"],{x:300, alpha:0});

	TweenLite.to(["#frame3_copy2"], 0.4, {x:0, ease: Quad.easeOut, alpha:1, delay:0});

	TweenLite.delayedCall(3,frame4Animation);
}
function frame4Animation(){
	TweenLite.set(["#disclaimer-copy"],{alpha:0});
	TweenLite.set(["#frame4-bg"],{x:300, alpha:0});
	TweenLite.set(["#frame4_copy1","#frame4_copy2","#delearName"],{x:300, alpha:0});


	TweenLite.to(["#frame3_copy1","#frame3_copy2","#frame2-bg"], 0.4, {x:-300, ease: Quad.easeOut,delay:0});
	TweenLite.to(["#frame4-bg","#frame4_copy1","#frame4_copy2","#delearName"], 0.4, {x:0, ease: Quad.easeOut, alpha:1, delay:0,onComplete:showvisibility});
	
	//TweenLite.delayedCall(3.5,frame5Animation);

}





// images to preload
var images = [ 
	'images/acura-logo.png',
	'images/acura-logo-2.png',
	'images/cta.png',
	'images/disclaimer-copy.png',
	'images/frame1_copy1.png',
	'images/frame1_copy2.png',
	'images/frame1_copy1.png',
	'images/f2_ilx_logo.png',
	'images/frame2-bg.jpg',
	'images/frame4-bg.jpg',
	'images/frame3_copy1.png',
	'images/frame3_copy2.png',
	
	'images/frame4_copy1.png',	
	'images/frame4_copy2.png',	
	
	
	
	
];

function preloadImages(urls, callback) {
	var length = urls.length, loaded = 0;
	for (var count = 0; count < length; count++) {

		var img = new Image();
		img.onload = function() {
			if (++loaded == length && callback) callback();
		}
		img.src = urls[count];
	}
}

function todofunc(){
	console.log('***run todo');
	startAnimation();
    addevents();
}

function initBanner(){
	console.log('***init');
	preloadImages(images, todofunc);
}


