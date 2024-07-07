import os
import sys
import webbrowser

DEFAULT_DIR = '/home/ori/data/simulation/v2.5/2023-07-23_dof_props_cpu/simulation/1/runs/cpu'


HTML_STYLE = '''
<style>
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 0;
      background-color: #f2f2f2;
    }

    .container {
      display: flex;
      max-width: 100vw;
      height: 100vh;
      margin: 0;
      padding: 0;
      background-color: #fff;
    }

    .left-pane {
      width: 20%;
      max-height: 100%;
      overflow-y: auto;
      margin: 0;
      padding: 0;
      border-right: 1px solid #ddd;
    }

    .right-pane {
      flex: 1;
      position: relative;
      display: flex;
      margin: 0;
      padding: 0;
      flex-direction: column;
    }

    #preview_frame {
      width: 100%;
      flex: 1;
      border: none;
    }

    .subtitle {
      font-size: 14px;
      margin: 10px;
      color: midnightblue;
    }

    .navigation-buttons {
      display: flex;
      justify-content: space-between;
      padding: 10px 20px;
      background-color: #f0f0f0;
    }

    .navigation-buttons button {
      margin-right: 10px;
    }

    .selected-link {
      background-color: #ffcc00;
      border-radius: 4px;
    }

    .subtitle.clicked {
      color: #000;
      font-weight: bold;
    }

    .separator {
      width: 4px;
      background-color: #f0f0f0;
    }

    .icon-button {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      width: 24px;
      height: 24px;
      border: none;
      background: none;
      cursor: pointer;
    }

    .icon-button:hover {
      background-color: #f0f0f0;
    }

    .icon-button svg {
      fill: #333;
      width: 100%;
      height: 100%;
    }

    .disabled-button {
      pointer-events: none;
      opacity: 0.5;
      cursor: not-allowed;
    }
  </style>
'''

RIGHT_PANE = '''
    <div class="right-pane">
      <p class="subtitle"><span id="preview_subtitle"></span></p>
      <iframe id="preview_frame" name="preview_frame"></iframe>
      <div class="navigation-buttons">
        <button id="previousBtn" onclick="showPreviousPreview()" class="icon-button">
          <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
            <path d="M15.41 7.41L14 6l-6 6 6 6 1.41-1.41L10.83 12z" />
          </svg>
        </button>
        <button id="nextBtn" onclick="showNextPreview()" class="icon-button">
          <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
            <path d="M8.59 16.59L10 18l6-6-6-6-1.41 1.41L13.17 12z" />
          </svg>
        </button>
      </div>
    </div>
'''

SCRIPT = '''
    <script>
      const previewLinks = document.querySelectorAll('[target="preview_frame"]');
      const subtitleElement = document.getElementById('preview_subtitle');
      const previousButton = document.getElementById('previousBtn');
      const nextButton = document.getElementById('nextBtn');
      const leftPane = document.querySelector('.left-pane');
      const separator = document.querySelector('.separator');
      const rightPane = document.querySelector('.right-pane');
      let currentPreviewIndex = 0;

      function formatSubtitle(subtitle) {
        return subtitle
          .split(' ').join('_').split('_')
          .map(word => word.charAt(0).toUpperCase() + word.slice(1))
          .join(' ');
      }

      function updateSubtitle(subtitle, index) {
        const formattedSubtitle = formatSubtitle(subtitle);
        subtitleElement.innerText = formattedSubtitle;

        const selectedLink = document.querySelector('.selected-link');
        if (selectedLink) {
          selectedLink.classList.remove('selected-link');
        }

        previewLinks[index].previousElementSibling.classList.add('selected-link');
        currentPreviewIndex = index;
        previousButton.disabled = currentPreviewIndex === 0;
        nextButton.disabled = currentPreviewIndex === previewLinks.length - 1;
        subtitleElement.classList.add('clicked');
      }

      function showPreviousPreview() {
        if (currentPreviewIndex > 0) {
          updateSubtitle(previewLinks[currentPreviewIndex - 1].previousElementSibling.innerText, currentPreviewIndex - 1);
          previewLinks[currentPreviewIndex].click();
        }
      }

      function showNextPreview() {
        if (currentPreviewIndex < previewLinks.length - 1) {
          updateSubtitle(previewLinks[currentPreviewIndex + 1].previousElementSibling.innerText, currentPreviewIndex + 1);
          previewLinks[currentPreviewIndex].click();
        }
      }

      /* updateSubtitle(previewLinks[0].previousElementSibling.innerText, 0); */
    </script>
'''

SEPARATOR = '''
    <div class="separator"></div>'''

link_counter = 0


def generate_html_directory(root, directory=None, show=True):
    if directory is None:
        directory = root
    output_html = os.path.join(directory, 'plots.html')
    if os.path.exists(output_html):
        os.remove(output_html)

    left_pane = f'<div class="left-pane">{generate_html_links(root, directory)}</div>'
    html_body = f'<body><div class="container">{left_pane}{SEPARATOR}{RIGHT_PANE}</div>{SCRIPT}</body>'
    html_head = f'<head>{HTML_STYLE}</head>'
    html_content = f'''<!DOCTYPE html>
<html>
{html_head}
{html_body}
</html>
'''

    with open(output_html, 'w') as file:
        file.write(html_content)

    print(f"Generated HTML directory at: {output_html}")
    if show:
        webbrowser.open(output_html)


def generate_html_links(root, directory, parent_path=''):
    global link_counter
    html_files = sorted([f for f in os.listdir(directory) if f.endswith('.html')])
    links = []

    for file_name in html_files:
        file_path = os.path.join(parent_path, directory, file_name)
        rel_path = file_path.replace(root, "")
        if rel_path.startswith("/"):
            rel_path = rel_path[1:]
        name = file_name.split('.')[0]
        link_title = f"<span>{name}</span>&nbsp;"
        rel_path_title = " -> ".join(os.path.split(rel_path)[0].split(os.sep)+[name])
        preview_link = f'<a href="{rel_path}" target="preview_frame" onclick="updateSubtitle(\'{rel_path_title}\', {link_counter})">preview</a>&nbsp;'
        open_link = f'<a href="{rel_path}">open</a>'
        link = f'<li><p>{link_title}{preview_link}{open_link}</p></li>'
        links.append(link)
        link_counter += 1

    if subdirectories := sorted(
        [
            d
            for d in os.listdir(directory)
            if os.path.isdir(os.path.join(directory, d))
        ]
    ):
        subdirectory_links = []
        for subdir in subdirectories:
            subdir_path = os.path.join(directory, subdir)
            subdir_html_content = generate_html_links(
                root, subdir_path, parent_path=os.path.join(parent_path, directory))
            subdirectory_links.append(f'<li>{subdir}{subdir_html_content}</li>')
            # subdirectory_links.append(f'<li>{subdir}<ul>{subdir_html_content}</ul></li>')
        links.extend(subdirectory_links)

    return '<ul>' + ''.join(links) + '</ul>'


if __name__ == '__main__':
    directory = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR
    generate_html_directory(directory)
