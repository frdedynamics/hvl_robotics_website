document.addEventListener("DOMContentLoaded", function() {
  var toggleButtons = document.querySelectorAll("#toggleButton");
  var hiddenTexts = document.querySelectorAll("#hiddenText");

  toggleButtons.forEach(function(button, index) {
    button.addEventListener("click", function() {
      var content = hiddenTexts[index];
      if (content) {
        if (content.style.display === "none") {
          content.style.display = "block";
        } else {
          content.style.display = "none";
        }
      }
    });
  });
});
  