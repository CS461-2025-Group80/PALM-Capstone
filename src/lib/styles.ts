/* for visual consistency... */

export const PARAGRAPH_STYLES = {
  base: 'text-base text-gray-700 leading-relaxed',
  lg: 'text-lg text-gray-700 leading-relaxed',
  sm: 'text-sm text-gray-600 leading-relaxed',
  muted: 'text-gray-600',
} as const;

export const SPACING = {
  paragraph: 'mb-4',
  sectionBottom: 'mb-8',
  sectionTop: 'mt-2',
} as const;

export const HEADING_STYLES = {
  h2: 'text-2xl font-semibold',
  h3: 'text-lg font-semibold',
} as const;

export const FIGURE_STYLES = {
  figure: 'flex flex-col items-center max-w-xs',
  image: 'w-auto h-40 object-cover rounded-md mb-4',
  image_robot: 'w-55 h-40 object-cover rounded-md mb-4',
  caption: `text-sm ${PARAGRAPH_STYLES.muted} italic text-center break-words`,
  grid: 'grid grid-cols-2 gap-6 my-6 mx-auto justify-items-center',
} as const;

export const CONTRIBUTOR_STYLES = {
  image: 'w-40 h-40 object-cover rounded-md mb-4 mx-auto',
  placeholder: 'w-40 h-40 bg-gray-200 rounded-md mb-4 mx-auto flex items-center justify-center',
} as const;

export const DESCRIPTION_PARAGRAPH = `${PARAGRAPH_STYLES.base} ${SPACING.paragraph}`;
export const LARGE_DESCRIPTION = `${PARAGRAPH_STYLES.lg} ${SPACING.paragraph}`;
export const MUTED_PARAGRAPH = `${PARAGRAPH_STYLES.muted} ${SPACING.sectionBottom} ${SPACING.sectionTop}`;
